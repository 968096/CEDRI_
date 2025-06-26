/**
 * BME688_Forced_Mode_MQTT.ino
 *
 * BME688 Multi-Sensor System using FORCED MODE for precise control
 * Manual control of 10-step heating profiles with synchronized measurements
 * With MQTT publishing capability for reliable data transmission
 *
 * Hardware: ESP32 Feather + BME688 Dev Kit (8 sensors)
 * Communication: I2C (pins 23,22) + SPI for sensor data
 */

#define MQTT_MAX_PACKET_SIZE 4096

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "bme68xLibrary.h"
#include <commMux.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "protobuf_handler.h"
#include "measurement.pb.h"

// ────────────────────────────────────────────────────────────────
// Configuration
// ────────────────────────────────────────────────────────────────
#define N_KIT_SENS       8
#define MAX_MEASUREMENTS 10
#define STEP_DURATION    5000  // ms
#define HEAT_STABILIZE   2000  // ms

// 🌐 NETWORK SETTINGS
const char* WIFI_SSID   = "Quiet House";
const char* WIFI_PASS   = "quiethouse2025@";
const char* MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT= 1883;
const char* MQTT_TOPIC  = "home/sensors/bme688_sequential101";

// Device metadata
const char* DEVICE_ID = "sensor1";
const char* LOCATION  = "disney";
const float  VOLUME_L = 1.5f;

// ────────────────────────────────────────────────────────────────
// Profiles (unchanged)
// ────────────────────────────────────────────────────────────────
uint16_t tempProfiles[N_KIT_SENS][MAX_MEASUREMENTS] = {
  {320,100,100,100,200,200,200,320,320,320},
  {100,100,200,200,200,200,320,320,320,320},
  {100,320,320,200,200,200,320,320,320,320},
  {100,320,320,200,200,200,320,320,320,320},
  {100,320,320,200,200,200,320,320,320,320},
  {100,320,320,200,200,200,320,320,320,320},
  { 50, 50,350,350,350,140,140,350,350,350},
  { 50, 50,350,350,350,140,140,350,350,350}
};

uint16_t durProfiles[N_KIT_SENS][MAX_MEASUREMENTS] = {
  {  700,  280, 1400, 4200,  700,  700,  700,  700,  700,  700},
  {  280, 5740,  280,  280, 1680, 1960, 1960,  280, 1960, 3920},
  { 6020,  280,  280,  280, 2940, 2940,  280, 1960, 1960, 1960},
  { 8960,  280,  280,  280, 4340, 4340,  280, 2800, 2940, 2940},
  { 6020,  280,  280,  280, 2940, 2940,  280, 1960, 1960, 1960},
  { 8960,  280,  280,  280, 4340, 4340,  280, 2800, 2940, 2940},
  { 9800, 9800,  140,  140,19320, 9800, 9800,  140,  140,19320},
  {14000,14000,  140,  140,27720,14000,14000,  140,  140,27720}
};

// ────────────────────────────────────────────────────────────────
// Only use the HP-names now
// ────────────────────────────────────────────────────────────────
const char* hpNames[N_KIT_SENS] = {
  "HP-354","HP-301","HP-321","HP-322",
  "HP-323","HP-324","HP-331","HP-332"
};

// ────────────────────────────────────────────────────────────────
// Hardware & RTOS objects
// ────────────────────────────────────────────────────────────────
Bme68x            bme[N_KIT_SENS];
comm_mux           commSetup[N_KIT_SENS];
WiFiClient        wifiClient;
PubSubClient      mqtt(wifiClient);
SemaphoreHandle_t spiMutex;

// Tracking
bool     sensorActive[N_KIT_SENS]  = {false};
uint32_t totalReadings[N_KIT_SENS] = {0};
uint32_t validReadings[N_KIT_SENS] = {0};
uint32_t profileCycles            = 0;
uint8_t  currentStep              = 0;

// ────────────────────────────────────────────────────────────────
// Print status
// ────────────────────────────────────────────────────────────────
void printStatus() {
  Serial.println("\n=== SENSOR STATUS ===");
  int active=0;
  uint32_t tot=0, val=0;
  for(uint8_t i=0;i<N_KIT_SENS;i++){
    Serial.printf("Sensor %u (%s): ", i, hpNames[i]);
    if(sensorActive[i]){
      active++; tot+=totalReadings[i]; val+=validReadings[i];
      Serial.printf("ACTIVE | Total:%u | Valid:%u | Cur:%u°C\n",
                    totalReadings[i], validReadings[i], tempProfiles[i][currentStep]);
    } else {
      Serial.println("INACTIVE");
    }
  }
  Serial.printf("\nActive %d/%d | Step:%u | Cycles:%u\n",
                active, N_KIT_SENS, currentStep, profileCycles);
  Serial.printf("Tot:%u | Valid:%u ", tot, val);
  if(tot) Serial.printf("| Success:%u%%\n",(val*100)/tot);
  Serial.printf("Heap:%u bytes | Uptime:%us\n",
                ESP.getFreeHeap(), millis()/1000);
  Serial.println("=====================\n");
}

// ────────────────────────────────────────────────────────────────
// WiFi / MQTT helpers
// ────────────────────────────────────────────────────────────────
void connectWiFi(){
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while(WiFi.status()!=WL_CONNECTED){Serial.print('.');delay(500);}
  Serial.println(" ✓");
}

void connectMQTT(){
  while(!mqtt.connected()){
    if(mqtt.connect("ESP32_BME688_CLIENT")) Serial.println("MQTT connected");
    else {Serial.printf("MQTT fail rc=%d\n",mqtt.state());delay(2000);}  }
}

// ────────────────────────────────────────────────────────────────
// Queue Protobuf for MQTT
// ────────────────────────────────────────────────────────────────
void queueProtobufForMQTT(uint8_t id, const char* profile, uint8_t step, float tc, float hu, float pr, uint32_t gr, bool gv, bool hs, uint64_t ts) {
  uint8_t protobufBuffer[MQTT_MAX_PACKET_SIZE];
  size_t messageLength;
  bool success = ProtobufHandler::packSensorReading(
    id, profile, step, tc, hu, pr, gr, gv, hs, ts,
    protobufBuffer, sizeof(protobufBuffer), &messageLength);

  if (success) {
    mqtt.publish(MQTT_TOPIC, protobufBuffer, messageLength);
  } else {
    Serial.println("Failed to encode Protobuf message");
  }
}

// ────────────────────────────────────────────────────────────────
// Trigger one forced-mode measurement
// ────────────────────────────────────────────────────────────────
void triggerMeasurement(uint8_t id,uint8_t step){
  if(!sensorActive[id]) return;
  uint16_t t=tempProfiles[id][step];
  uint16_t d=durProfiles[id][step];
  bme[id].setHeaterProf(t,d);
  bme[id].setOpMode(BME68X_FORCED_MODE);
  uint32_t us=bme[id].getMeasDur(BME68X_FORCED_MODE);
  delay((us+999)/1000);
}

// ────────────────────────────────────────────────────────────────
// Collect data and queue Protobuf
// ────────────────────────────────────────────────────────────────
void collectResults(uint8_t step){
  for(uint8_t i=0;i<N_KIT_SENS;i++){
    if(!sensorActive[i]) continue;
    if(bme[i].fetchData()){
      bme68xData data;
      uint8_t left;
      do {
        left = bme[i].getData(data);
        if(data.status & BME68X_NEW_DATA_MSK){
          totalReadings[i]++;
          bool gv = data.status & BME68X_GASM_VALID_MSK;
          bool hs = data.status & BME68X_HEAT_STAB_MSK;
          if(gv && hs) validReadings[i]++;
          queueProtobufForMQTT(i, hpNames[i], step, data.temperature, data.humidity, data.pressure, data.gas_resistance, gv, hs, millis());
        }
      } while(left);
    }
  }
}

// ────────────────────────────────────────────────────────────────
// MQTT Task
// ────────────────────────────────────────────────────────────────
void mqttTask(void*){
  connectWiFi(); connectMQTT();
  while(true){
    mqtt.loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ────────────────────────────────────────────────────────────────
// Measurement Task
// ────────────────────────────────────────────────────────────────
void measurementTask(void*){
  TickType_t lastWake = xTaskGetTickCount();
  for(;;){
    collectResults(currentStep);
    currentStep = (currentStep+1)%MAX_MEASUREMENTS;
    if(currentStep==0){
      profileCycles++;
      Serial.printf("=== Completed cycle %u ===\n\n",profileCycles);
    }
    Serial.printf("Starting step %u…\n",currentStep);
    for(uint8_t i=0;i<N_KIT_SENS;i++){
      if(sensorActive[i]) triggerMeasurement(i,currentStep);
    }
    delay(HEAT_STABILIZE);

    // Debug: print stack usage
    UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("Measure task stack high water mark: %u bytes\n", stackHighWater * sizeof(StackType_t));

    vTaskDelayUntil(&lastWake,pdMS_TO_TICKS(STEP_DURATION));
  }
}

// ────────────────────────────────────────────────────────────────
// Status Task
// ──────────────────────────────────────────────────                               ──────────────
void statusTask(void*){
  for(;;){
    vTaskDelay(pdMS_TO_TICKS(60000));
    printStatus();
  }
}

// ────────────────────────────────────────────────────────────────
// setup() and loop()
// ────────────────────────────────────────────────────────────────
void setup(){
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("=== BME688 Forced Mode + MQTT (RTOS) ===");

  Wire.begin(23,22);
  Wire.setClock(400000);
  SPI.begin();
  comm_mux_begin(Wire,SPI);

  // init sensors
  for(uint8_t i=0;i<N_KIT_SENS;i++){
    commSetup[i]=comm_mux_set_config(Wire,SPI,i,commSetup[i]);
    bme[i].begin(BME68X_SPI_INTF,
                 comm_mux_read,comm_mux_write,comm_mux_delay,
                 &commSetup[i]);
    if(bme[i].checkStatus()) sensorActive[i]=false;
    else { bme[i].setTPH(); bme[i].setOpMode(BME68X_FORCED_MODE); sensorActive[i]=true; }
  }

  mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);
  mqtt.setServer(MQTT_BROKER,MQTT_PORT);
  mqtt.setKeepAlive(60);

  spiMutex  = xSemaphoreCreateMutex();

  // Increase stack size for measurement task to avoid overflow
  const uint32_t MEASURE_TASK_STACK = 12288; // 12 KB
  xTaskCreatePinnedToCore(measurementTask, "Measure", MEASURE_TASK_STACK, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask,       "MQTT",    4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(statusTask,     "Status",  2048, NULL, 1, NULL, 1);
}

void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}