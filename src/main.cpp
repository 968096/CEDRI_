#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "bme68xLibrary.h"
#include "commMux.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ──────────────────────────────────────────────────────────────────────────────
// build‑time constants
// ──────────────────────────────────────────────────────────────────────────────
#define N_KIT_SENS           8          // dev‑kit has 8 BME688s
#define BASE_HEAT_DUR_MS  1000          // used to derive heatDur for each profile
#define MAX_MEASUREMENTS     10
#define MIN_CYCLE_MEASUREMENTS 5

// Wi‑Fi / MQTT
const char* WIFI_SSID   = "Quiet House";
const char* WIFI_PASS   = "quiethouse2025@";
const char* MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TOPIC  = "home/sensors/bme688_hierarchical";

// Device information
const char* DEVICE_ID = "ESP32_BME688_CAFE_001";
const char* LOCATION   = "Coffee_Lab_Station_1";
const float  VOLUME_L  = 1.5;

// ──────────────────────────────────────────────────────────────────────────────
// per‑sensor heater profiles (10 steps)   temp (°C) / cumulative time (ms)
// ──────────────────────────────────────────────────────────────────────────────
uint16_t tempProfiles[N_KIT_SENS][10] = {
  {320,100,100,100,200,200,200,320,320,320}, // HP‑354
  {100,100,200,200,200,200,320,320,320,320}, // HP‑301
  {100,320,320,200,200,200,320,320,320,320}, // HP‑321
  {100,320,320,200,200,200,320,320,320,320}, // HP‑322
  {100,320,320,200,200,200,320,320,320,320}, // HP‑323
  {100,320,320,200,200,200,320,320,320,320}, // HP‑324
  { 50, 50,350,350,350,140,140,350,350,350}, // HP‑331
  { 50, 50,350,350,350,140,140,350,350,350}  // HP‑332
};

const uint32_t cumTimes[N_KIT_SENS][10] = {
  {  700,   980,  2380,  6580,  7280,  7980,  8680,  9380, 10080, 10780},
  {  280,  6020,  6300,  6580,  8260, 10220, 12180, 12460, 14420, 18340},
  { 6020,  6300,  6580,  6860,  9800, 12740, 13020, 14980, 16940, 18900},
  { 8960,  9240,  9520,  9800, 14140, 18480, 18760, 21560, 24500, 27440},
  { 6020,  6300,  6580,  6860,  9800, 12740, 13020, 14980, 16940, 18900},
  { 8960,  9240,  9520,  9800, 14140, 18480, 18760, 21560, 24500, 27440},
  { 9800, 19600, 19740, 19880, 39200, 49000, 58800, 58940, 59080, 78400},
  {14000, 28000, 28140, 28280, 56000, 70000, 84000, 84140, 84280,112000}
};

const char* hpNames[N_KIT_SENS] = {
  "HP-354","HP-301","HP-321","HP-322","HP-323","HP-324","HP-331","HP-332"
};

// ──────────────────────────────────────────────────────────────────────────────
// hardware objects
// ──────────────────────────────────────────────────────────────────────────────
WiFiClient     wifiClient;
PubSubClient   mqtt(wifiClient);
Bme68x         bme[N_KIT_SENS];
commMux        commSetup[N_KIT_SENS];

// ──────────────────────────────────────────────────────────────────────────────
// FreeRTOS containers
// ──────────────────────────────────────────────────────────────────────────────
QueueHandle_t  sensorQueue = NULL;
QueueHandle_t  jsonQueue = NULL;
SemaphoreHandle_t spiMutex = NULL, mqttMutex = NULL;

// ──────────────────────────────────────────────────────────────────────────────
// application‑level data structures
// ──────────────────────────────────────────────────────────────────────────────
typedef struct {
  int  measurement_step;
  int  set_temp_c;
  float temp_c;
  float humidity_pct;
  float pressure_hpa;
  uint32_t gas_resistance_ohm;
  bool gas_valid;
  bool heat_stable;
  uint32_t timestamp;
} Measurement_t;

typedef struct {
  int  sensor_id;
  char heater_profile[10];
  Measurement_t measurements[MAX_MEASUREMENTS];
  int  measurement_count;
  int  last_step_received;
  bool ready_to_send;
  uint32_t last_update;
  bool cycle_active;
} SensorData_t;

typedef struct {
  char device_id[32];
  char location[32];
  float volume_l;
  int   battery_pct;
  SensorData_t sensors[N_KIT_SENS];
} DeviceState_t;

typedef struct {
  uint32_t   timestamp;
  uint8_t    sensor_id;
  bme68xData data;
  uint8_t    profile_step;
} SensorReading_t;

typedef struct {
  char   payload[2048];
  size_t length;
} JsonMessage_t;

DeviceState_t deviceState;

// ──────────────────────────────────────────────────────────────────────────────
// per‑sensor timing (calculated at setup)
// ──────────────────────────────────────────────────────────────────────────────
uint32_t measPeriod[N_KIT_SENS];   // ms between consecutive fetchData() calls

// Tick snapshot for each sensor
static TickType_t lastWake[N_KIT_SENS];

// ──────────────────────────────────────────────────────────────────────────────
// helper functions
// ──────────────────────────────────────────────────────────────────────────────
void initDeviceState() {
  strncpy(deviceState.device_id, DEVICE_ID, sizeof(deviceState.device_id));
  strncpy(deviceState.location , LOCATION , sizeof(deviceState.location ));
  deviceState.volume_l    = VOLUME_L;
  deviceState.battery_pct = 0;

  for (uint8_t i = 0; i < N_KIT_SENS; ++i) {
    deviceState.sensors[i].sensor_id = i;
    strncpy(deviceState.sensors[i].heater_profile, hpNames[i], sizeof(deviceState.sensors[i].heater_profile));
    deviceState.sensors[i].measurement_count  = 0;
    deviceState.sensors[i].last_step_received = -1;
    deviceState.sensors[i].ready_to_send      = false;
    deviceState.sensors[i].last_update        = 0;
    deviceState.sensors[i].cycle_active       = false;
  }
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("\nWiFi connected");
}

void connectMQTT() {
  while (!mqtt.connected()) {
    if (mqtt.connect("ESP32_BME688_Hierarchical")) {
      Serial.println("MQTT connected");
      break;
    }
    Serial.print("MQTT connection failed, rc=");
    Serial.println(mqtt.state());
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// reset measurements of a sensor after a full cycle is sent
void resetSensorData(SensorData_t *sensor) {
  sensor->measurement_count  = 0;
  sensor->ready_to_send      = false;
  sensor->cycle_active       = false;

  for (int j = 0; j < MAX_MEASUREMENTS; ++j) {
    sensor->measurements[j] = Measurement_t{};  // zero‑initialise
  }
}

void sendSensorData(SensorData_t *sensor) {
  DynamicJsonDocument doc(2048);
  JsonMessage_t jsonMsg;

  JsonObject device = doc.createNestedObject("device");
  device["device_id"]   = deviceState.device_id;
  device["location"]    = deviceState.location;
  device["volume_l"]    = deviceState.volume_l;
  device["battery_pct"] = deviceState.battery_pct;

  JsonArray sensorsArray = device.createNestedArray("sensors");
  JsonObject sensorObj   = sensorsArray.createNestedObject();
  sensorObj["sensor_id"]      = sensor->sensor_id;
  sensorObj["heater_profile"] = sensor->heater_profile;

  JsonArray measurementsArray = sensorObj.createNestedArray("measurements");
  for (int j = 0; j < MAX_MEASUREMENTS; ++j) {
    if (sensor->measurements[j].measurement_step > 0) {
      const Measurement_t *meas = &sensor->measurements[j];
      JsonObject mObj = measurementsArray.createNestedObject();
      mObj["measurement_step"]   = meas->measurement_step;
      mObj["set_temp_c"]         = meas->set_temp_c;
      mObj["temp_c"]             = round(meas->temp_c * 100) / 100.0;
      mObj["humidity_pct"]       = round(meas->humidity_pct * 100) / 100.0;
      mObj["pressure_hpa"]       = round(meas->pressure_hpa * 100) / 100.0;
      mObj["gas_resistance_ohm"] = meas->gas_resistance_ohm;
      mObj["gas_valid"]          = meas->gas_valid;
      mObj["heat_stable"]        = meas->heat_stable;
      mObj["timestamp"]          = meas->timestamp;
    }
  }

  jsonMsg.length = serializeJson(doc, jsonMsg.payload, sizeof(jsonMsg.payload));

  if (jsonMsg.length > 0 && jsonMsg.length < sizeof(jsonMsg.payload)) {
    if (jsonQueue != NULL) {
      xQueueSend(jsonQueue, &jsonMsg, pdMS_TO_TICKS(100));
    }
    Serial.printf("Sensor %d: JSON ready (%d bytes, %d measurements)\n",
                  sensor->sensor_id, jsonMsg.length, measurementsArray.size());
  } else {
    Serial.printf("Sensor %d: JSON error, size %d\n", sensor->sensor_id, jsonMsg.length);
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// RTOS Tasks
// ──────────────────────────────────────────────────────────────────────────────
void sensorTask(void* /*pv*/) {
  SensorReading_t reading;
  // initialise lastWake array once inside task context
  for (uint8_t i = 0; i < N_KIT_SENS; ++i) lastWake[i] = xTaskGetTickCount();

  while (true) {
    TickType_t now = xTaskGetTickCount();

    for (uint8_t i = 0; i < N_KIT_SENS; ++i) {
      // wait for this sensor's profile period to elapse
      if ((now - lastWake[i]) < pdMS_TO_TICKS(measPeriod[i])) continue;
      lastWake[i] = now;

      if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(50))) {
        if (bme[i].fetchData()) {
          int8_t n;
          do {
            n = bme[i].getData(reading.data);
            if (n > 0 && (reading.data.status & BME68X_NEW_DATA_MSK)) {
              reading.timestamp    = millis();
              reading.sensor_id    = i;
              reading.profile_step = reading.data.meas_index; // 0‑9 in sequential
              if (sensorQueue != NULL) {
                xQueueSend(sensorQueue, &reading, 0);
              }
            }
          } while (n > 0);   // drain FIFO completely
        }
        xSemaphoreGive(spiMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));  // cooperative multitasking
  }
}

void jsonAggregatorTask(void* /*pv*/) {
  SensorReading_t reading;

  while (true) {
    if (sensorQueue != NULL && xQueueReceive(sensorQueue, &reading, pdMS_TO_TICKS(100))) {
      if (reading.sensor_id >= N_KIT_SENS) {
        Serial.printf("ERRO: sensor_id fora do range: %u\n", reading.sensor_id);
        continue;
      }
      SensorData_t *sensor = &deviceState.sensors[reading.sensor_id];

      // start a new cycle exclusively at step 0
      if (!sensor->cycle_active && reading.profile_step == 0) {
        sensor->cycle_active = true;
        Serial.printf("Sensor %d: new cycle\n", reading.sensor_id);
      }

      // detect cycle completion (step 9 → 0)
      bool cycle_completed = false;
      if (sensor->cycle_active && sensor->last_step_received != -1 &&
          reading.profile_step == 0 && sensor->last_step_received == 9) {
        cycle_completed = true;
      }

      // send and reset before processing the new step 0 frame
      if (cycle_completed && sensor->measurement_count >= MIN_CYCLE_MEASUREMENTS) {
        sendSensorData(sensor);
        resetSensorData(sensor);
        sensor->cycle_active = true; // keep logging
      }

      // store measurement if cycle active
      if (sensor->cycle_active && reading.profile_step < MAX_MEASUREMENTS) {
        Measurement_t *meas = &sensor->measurements[reading.profile_step];
        meas->measurement_step   = reading.profile_step + 1;
        meas->set_temp_c         = tempProfiles[reading.sensor_id][reading.profile_step];
        meas->temp_c             = reading.data.temperature;
        meas->humidity_pct       = reading.data.humidity;
        meas->pressure_hpa       = reading.data.pressure;
        meas->gas_resistance_ohm = reading.data.gas_resistance;
        meas->gas_valid          = reading.data.status & BME68X_GASM_VALID_MSK;
        meas->heat_stable        = reading.data.status & BME68X_HEAT_STAB_MSK;
        meas->timestamp          = reading.timestamp;

        if (reading.profile_step >= sensor->measurement_count) {
          sensor->measurement_count = reading.profile_step + 1;
        }
      }

      sensor->last_step_received = reading.profile_step;
      sensor->last_update        = millis();
    }
  }
}

void mqttTask(void* /*pv*/) {
  JsonMessage_t jsonMsg;

  while (true) {
    if (jsonQueue != NULL && xQueueReceive(jsonQueue, &jsonMsg, portMAX_DELAY)) {
      if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(2000))) {
        if (WiFi.status() != WL_CONNECTED)   connectWiFi();
        if (!mqtt.connected())               connectMQTT();

        if (mqtt.connected()) {
          bool ok = mqtt.publish(MQTT_TOPIC, jsonMsg.payload, false);
          Serial.printf("MQTT: publish %s (%u bytes)\n", ok ? "ok" : "fail", (unsigned)jsonMsg.length);
        }
        mqtt.loop();
        xSemaphoreGive(mqttMutex);
      }
    }
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// Arduino setup / loop
// ──────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(23, 22);    // SDA, SCL
  Serial.println("BME688 multi‑profile system (sequential mode)");

  initDeviceState();

  // initialise comm mux e sensores
  commMuxBegin(Wire, SPI);

  for (uint8_t i = 0; i < N_KIT_SENS; ++i) {
    commSetup[i] = commMuxSetConfig(Wire, SPI, i, commSetup[i]);
    bme[i].begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &commSetup[i]);

    if (bme[i].checkStatus()) {
      Serial.printf("Sensor %d (%s) init failed\n", i, hpNames[i]);
      continue;
    }

    bme[i].setTPH();   // humidity / pressure / temperature oversampling defaults

    // ── calcula heatDur e multiplicadores para este sensor ────────────────
    uint32_t tph_us  = bme[i].getMeasDur(BME68X_SEQUENTIAL_MODE);  // µs
    uint16_t tph_ms  = (tph_us + 999) / 1000;                      // round up
    uint16_t heatDur = BASE_HEAT_DUR_MS - tph_ms;                  // base slice (ms)

    uint16_t mulProfile[10];
    for (uint8_t s = 0; s < 10; ++s) {
      uint32_t delta = (s == 0) ? cumTimes[i][0] : cumTimes[i][s] - cumTimes[i][s - 1];
      mulProfile[s]  = (delta + heatDur / 2) / heatDur;            // nearest integer ratio
      if (mulProfile[s] == 0) mulProfile[s] = 1;                   // nunca zero
    }

    bme[i].setHeaterProf(tempProfiles[i], mulProfile, heatDur, 10);
    bme[i].setOpMode(BME68X_SEQUENTIAL_MODE);

    // período total que esse perfil vai tomar
    uint32_t sumMul = 0; for (uint8_t s = 0; s < 10; ++s) sumMul += mulProfile[s];
    measPeriod[i] = tph_ms + (uint32_t)heatDur * sumMul;

    Serial.printf("Sensor %u (%s) configurado: heatDur %ums, period %ums\n",
                  i, hpNames[i], heatDur, measPeriod[i]);
  }

  mqtt.setBufferSize(2048);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(60);

  sensorQueue = xQueueCreate(20, sizeof(SensorReading_t));
  jsonQueue   = xQueueCreate( 8, sizeof(JsonMessage_t));
  spiMutex    = xSemaphoreCreateMutex();
  mqttMutex   = xSemaphoreCreateMutex();

  // Verificação das filas e semáforos
  if (!sensorQueue || !jsonQueue || !spiMutex || !mqttMutex) {
    Serial.println("Erro ao criar filas ou semáforos!");
    while(1); // trava para debug
  }

  connectWiFi();
  connectMQTT();

  xTaskCreatePinnedToCore(sensorTask         , "SensorTask" , 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(jsonAggregatorTask , "JsonTask"   , 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(mqttTask           , "MqttTask"   , 4096, NULL, 2, NULL, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}