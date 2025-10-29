#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <commMux.h>
#include "bme68xLibrary.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "pb_encode.h"
#include "measurement.pb.h"
#include "mbedtls/base64.h"

// ===================== CONFIG =====================
#define N_SENSORS       8
#define N_STEPS         10
#define N_RUNS          5
#define SLEEP_MINUTES   5
#define HEAT_STABILIZE_MS 700

// 🌐 WiFi + MQTT
const char* WIFI_SSID   = "Quiet House1";
const char* WIFI_PASS   = "quiethouse2025@";
const char* MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TOPIC  = "application/test/caio/bme688";

const uint16_t DEVICE_ID   = 1;
const uint16_t LOCATION_ID = 1;

// ===================== HEATER PROFILES =====================
static const uint16_t tempBase[N_SENSORS][N_STEPS] = {
  {100,320,170,320,240,240,240,320,320,320},
  {100,320,170,320,240,240,240,320,320,320},
  { 70,350,163,350,256,256,256,350,350,350},
  { 70,350,163,350,256,256,256,350,350,350},
  {210,265,265,320,320,265,210,155,100,155},
  {210,265,265,320,320,265,210,155,100,155},
  {210,280,280,350,350,280,210,140, 70,140},
  {210,280,280,350,350,280,210,140, 70,140}
};

static const uint32_t durBaseAbs[N_SENSORS][N_STEPS] = {
  {6020,6300,12320,12600,12880,15680,18620,18900,21700,24640},
  {8960,9240,18200,18480,18760,23100,27580,27860,32200,36680},
  {6020,6300,12320,12600,12880,15680,18620,18900,21700,24640},
  {8960,9240,18200,18480,18760,23100,27580,27860,32200,36680},
  {3360,3640,6720,7000,10080,13440,16800,20160,23520,26880},
  {4480,4760,8960,9240,13440,17920,22400,26880,31360,35840},
  {3360,3640,6720,7000,10080,13440,16800,20160,23520,26880},
  {4480,4760,8960,9240,13440,17920,22400,26880,31360,35840}
};

// ===================== GLOBALS =====================
Bme68x bme[N_SENSORS];
comm_mux commSetup[N_SENSORS];
bool sensorActive[N_SENSORS] = {false};
SemaphoreHandle_t busMutex;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===================== HELPERS =====================
static void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf(" ✓  IP: %s\n", WiFi.localIP().toString().c_str());
}

static void connectMQTT() {
  while (!mqtt.connected()) {
    String cid = "ESP32_BME688_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str())) Serial.println("MQTT connected");
    else { Serial.printf("MQTT fail rc=%d\n", mqtt.state()); delay(1000); }
  }
}

// ===================== MQTT / PROTOBUF =====================
static bool publishVOC(uint8_t sensor_idx, uint8_t step, const bme68xData &d) {
  if (WiFi.status() != WL_CONNECTED || !mqtt.connected()) return false;

  cedri_VOCReading proto = cedri_VOCReading_init_zero;
  proto.device_id          = DEVICE_ID;
  proto.sensor_id          = sensor_idx;
  proto.heater_profile     = (cedri_HeaterProfile)sensor_idx;
  proto.gas_resistance_ohm = d.gas_resistance;

  uint8_t buf[256];
  pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&os, cedri_VOCReading_fields, &proto)) return false;

  uint8_t out_buf[400]; size_t out_len = 0;
  if (mbedtls_base64_encode(out_buf, sizeof(out_buf), &out_len, buf, os.bytes_written) != 0) return false;

  String json = "{\"data\":\"" + String((char*)out_buf, out_len) + "\"}";
  mqtt.publish(MQTT_TOPIC, json.c_str(), false);
  return true;
}

// ===================== MEASUREMENT TASK =====================
static void measurementTask(void*) {
  vTaskDelay(pdMS_TO_TICKS(1000));

  for (uint8_t run = 0; run < N_RUNS; run++) {
    Serial.printf("\n🚀 Run %u/%u started...\n", run + 1, N_RUNS);

    for (uint8_t step = 0; step < N_STEPS; step++) {
      uint32_t maxDur = 0;
      Serial.printf("[Step %u/%u]\n", step + 1, N_STEPS);

      // --------- Força medição em modo FORCED -----------
      for (uint8_t i = 0; i < N_SENSORS; i++) {
        if (!sensorActive[i]) continue;

        if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(400))) {
          uint16_t temp = tempBase[i][step];
          uint32_t dur  = durBaseAbs[i][step];
          bme[i].setHeaterProf(temp, dur);
          bme[i].setOpMode(BME68X_FORCED_MODE);
          if (dur > maxDur) maxDur = dur;
          xSemaphoreGive(busMutex);
        }
      }

      // Aguarda tempo máximo + estabilização
      vTaskDelay(pdMS_TO_TICKS(maxDur + HEAT_STABILIZE_MS));

      // --------- Coleta e envia resultados -------------
      for (uint8_t i = 0; i < N_SENSORS; i++) {
        if (!sensorActive[i]) continue;
        if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(400))) {
          bme68xData d{};
          if (bme[i].fetchData() && (bme[i].getData(d), d.status & BME68X_NEW_DATA_MSK)) {
            publishVOC(i, step, d);
            Serial.printf("Sensor %u | T=%.1f°C | H=%.1f%% | P=%.1f hPa | Rgas=%.0fΩ\n",
                          i, d.temperature, d.humidity, d.pressure, d.gas_resistance);
          }
          xSemaphoreGive(busMutex);
        }
      }
    }
    Serial.printf("✅ Run %u/%u complete.\n", run + 1, N_RUNS);
  }

  Serial.println("🛌 Deep sleep for 5 minutes...");
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_sleep_enable_timer_wakeup(SLEEP_MINUTES * 60ULL * 1000000ULL);
  esp_deep_sleep_start();
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n=== BME688 Forced Mode | 10 steps × 5 runs ===");

  busMutex = xSemaphoreCreateMutex();

  connectWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(60);
  mqtt.setBufferSize(2048);
  connectMQTT();

  Wire.begin(23, 22);
  Wire.setClock(400000);
  SPI.begin();
  comm_mux_begin(Wire, SPI);

  Serial.println("Initializing 8x BME688...");
  for (uint8_t i = 0; i < N_SENSORS; i++) {
    vTaskDelay(pdMS_TO_TICKS(50));
    commSetup[i] = comm_mux_set_config(Wire, SPI, i, commSetup[i]);
    bme[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &commSetup[i]);

    if (bme[i].checkStatus() == 0) {
      bme[i].setTPH(BME68X_OS_2X, BME68X_OS_16X, BME68X_OS_1X);
      bme[i].setOpMode(BME68X_FORCED_MODE);
      sensorActive[i] = true;
      Serial.printf("✓ Sensor %u ready\n", i);
    } else {
      Serial.printf("✗ Sensor %u failed: %s\n", i, bme[i].statusString());
    }
  }

  xTaskCreatePinnedToCore(measurementTask, "Measure", 8192, NULL, 2, NULL, 1);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) connectMQTT();
    mqtt.loop();
  }
  delay(20);
}
