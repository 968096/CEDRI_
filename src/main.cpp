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
#include <../lib/demo/src/commMux.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

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
// <-- now publishing to “100” -->
const char* MQTT_TOPIC  = "home/sensors/bme688_sequential11";

// Device metadata
const char* DEVICE_ID = "ESP32_BME688_CAFE_001";
const char* LOCATION  = "Coffee_Lab_Station_1";
const float  VOLUME_L = 1.5f;

// ────────────────────────────────────────────────────────────────
// Profiles (unchanged)
// ────────────────────────────────────────────────────────────────
uint16_t tempProfiles[N_KIT_SENS][MAX_MEASUREMENTS] = {
  {100,320,170,320,240,240,240,320,320,320},
  {100,320,170,320,240,240,240,320,320,320},
  {70,350,163,350,256,256,256,350,250,350},
  {70,350,163,350,256,256,256,350,350,350},
  {210,265,265,320,320,265,210,155,100,155},
  {210,280,280,350,350,280,210,140,70,140},
  {210,280,280,350,350,280,210,140,70,140},
  { 210,280,280,350,350,280,210,140,70,140}
};

uint16_t mulProfiles[N_KIT_SENS][MAX_MEASUREMENTS] = {
  { 5, 2,10,30, 5, 5, 5, 5, 5, 5},
  { 3, 2, 8,25, 4, 4, 4, 4, 4, 4},
  { 7, 2,12,35, 6, 6, 6, 6, 6, 6},
  { 4, 3, 9,20, 5, 5, 5, 5, 5, 5},
  { 6, 4,11,28, 7, 7, 7, 7, 7, 7},
  { 8, 1,15,40, 8, 8, 8, 8, 8, 8},
  { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5},
  { 2, 4, 6, 8,10,12,14,16,18,20}
};

uint16_t durProfiles[N_KIT_SENS][MAX_MEASUREMENTS] = {
  {  6020,  6300, 12320, 12600,  12880,  15680,  18620,  18900,  21700,  24640},
  {  8960, 9240,  18200,  18480, 18760, 23100, 27580,  27860, 32200, 36680},
  { 6020,  6300,  12320,  12600, 12880, 15680,  18620, 18900, 21700, 24640},
  { 8960,  9240, 18200,  18480, 18760, 23100,  27580, 27860, 32200, 36680},
  { 3360,  3640,  6720,  7000, 10080, 13440,  16800, 20160, 23520, 26880},
  { 4480,  4760,  8960,  9240, 13440, 17920,  22400, 26880, 31360, 35840},
  { 3360, 3640,  6720,  7000,1008, 13440, 16800,  20160,  23520,26880},
  {4480,4760, 8960, 9240,13440,17920,22400,  26880,  31360,35840}
};

// ────────────────────────────────────────────────────────────────
// Only use the HP-names now
// ────────────────────────────────────────────────────────────────
const char* hpNames[N_KIT_SENS] = {
  "HP-411","HP-412","HP-413","HP-414",
  "HP-501","HP-502","HP-503","HP-504"
};

// ────────────────────────────────────────────────────────────────
// Hardware & RTOS objects
// ────────────────────────────────────────────────────────────────
Bme68x            bme[N_KIT_SENS];
comm_mux           commSetup[N_KIT_SENS];
WiFiClient        wifiClient;
PubSubClient      mqtt(wifiClient);
QueueHandle_t     csvQueue;
SemaphoreHandle_t spiMutex;

// Tracking
bool     sensorActive[N_KIT_SENS]  = {false};
uint32_t totalReadings[N_KIT_SENS] = {0};
uint32_t validReadings[N_KIT_SENS] = {0};
uint32_t profileCycles            = 0;
uint8_t  currentStep              = 0;
uint32_t stepStartTime            = 0;

// CSV message
struct CSVMessage_t {
  char   payload[256];
  size_t len;
};

// ────────────────────────────────────────────────────────────────
// Print status: now shows only HP-names
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
    else {Serial.printf("MQTT fail rc=%d\n",mqtt.state());delay(2000);}
  }
}

// ────────────────────────────────────────────────────────────────
// Queue CSV for MQTT
// ────────────────────────────────────────────────────────────────
void queueCSV(uint8_t id, float tc,float hu,float pr,uint32_t gr,
              bool gv,bool hs,uint8_t step,uint32_t ts){
  CSVMessage_t m;
  int n = snprintf(m.payload,sizeof(m.payload),
    "%s,%s,%.2f,%u,%s,%u,%.2f,%.2f,%.2f,%u,%u,%u,%u",
    DEVICE_ID,LOCATION,VOLUME_L,
    id, hpNames[id], step+1,
    tc,hu,pr,gr, gv?1:0, hs?1:0, ts);
  if(n<0) return;
  m.len=n;
  xQueueSend(csvQueue,&m,portMAX_DELAY);
}

// ────────────────────────────────────────────────────────────────
// MQTT task
// ────────────────────────────────────────────────────────────────
void mqttTask(void*){
  CSVMessage_t msg;
  TickType_t last = xTaskGetTickCount();
  connectWiFi(); connectMQTT();
  while(true){
    mqtt.loop();
    while(xQueueReceive(csvQueue,&msg,0)==pdTRUE){
      Serial.printf("[MQTT] Publishing to %s: %s\n",MQTT_TOPIC,msg.payload);
      bool ok;
      do{
        ok = mqtt.publish(MQTT_TOPIC,msg.payload,msg.len);
        mqtt.loop();
        if(!ok) vTaskDelay(pdMS_TO_TICKS(100));
      }while(!ok);
      Serial.printf("[MQTT] Published %u bytes\n",msg.len);
    }
    vTaskDelayUntil(&last,pdMS_TO_TICKS(10));
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
// Collect data (fixed do/while)
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
          // print raw CSV to Serial
          Serial.printf("%u,%s,%u,%u,%u,%.2f,%.2f,%.2f,%u,%u,%u,%u\n",
            i, hpNames[i], step, tempProfiles[i][step],
            data.temperature,data.humidity,data.pressure,
            data.gas_resistance, gv?1:0, hs?1:0, data.status);
          queueCSV(i,data.temperature,data.humidity,
                   data.pressure,data.gas_resistance,
                   gv,hs,step,millis());
        }
      } while(left);
    }
  }
}

// ────────────────────────────────────────────────────────────────
// setup()
void setup(){
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("=== BME688 Forced Mode + MQTT ===");

  Wire.begin(23,22);
  Wire.setClock(400000);
  SPI.begin();
  comm_mux_begin(Wire,SPI);
  Serial.println("commMux initialized");

  // optional expander test
  Wire.beginTransmission(0x20);
  Serial.println(Wire.endTransmission()==0?"Expander OK":"Expander FAIL");

  // init sensors
  for(uint8_t i=0;i<N_KIT_SENS;i++){
    Serial.printf("Init %s… ", hpNames[i]);
    commSetup[i]=comm_mux_set_config(Wire,SPI,i,commSetup[i]);
    bme[i].begin(BME68X_SPI_INTF,
                 comm_mux_read,comm_mux_write,comm_mux_delay,
                 &commSetup[i]);
    if(bme[i].checkStatus()){
      Serial.println("FAIL"); sensorActive[i]=false;
    } else {
      bme[i].setTPH();
      bme[i].setOpMode(BME68X_FORCED_MODE);
      sensorActive[i]=true;
      Serial.println("OK");
    }
  }

  mqtt.setBufferSize(4096);
  mqtt.setServer(MQTT_BROKER,MQTT_PORT);
  mqtt.setKeepAlive(60);

  csvQueue = xQueueCreate(64,sizeof(CSVMessage_t));
  spiMutex  = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(mqttTask,"MqttTask",4096,NULL,1,NULL,1);

  printStatus();
  Serial.println("Starting profile cycling…");
  stepStartTime = millis();
}

// ────────────────────────────────────────────────────────────────
// loop()
void loop(){
  static uint32_t lastStatus=0;
  if(millis()-stepStartTime >= STEP_DURATION){
    collectResults(currentStep);
    currentStep = (currentStep+1)%MAX_MEASUREMENTS;
    if(currentStep==0){
      profileCycles++;
      Serial.printf("=== Completed cycle %u ===\n\n",profileCycles);
    }
    Serial.printf("Starting step %u…\n",currentStep);
    for(uint8_t i=0;i<N_KIT_SENS;i++)
      if(sensorActive[i])
        triggerMeasurement(i,currentStep);
    delay(HEAT_STABILIZE);
    stepStartTime=millis();
  }
  if(millis()-lastStatus>60000){
    printStatus();
    lastStatus=millis();
  }
  delay(100);
}