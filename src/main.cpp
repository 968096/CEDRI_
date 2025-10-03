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
#include <math.h>

// ===================== CONFIG =====================
#define N_KIT_SENS         8
#define N_STEPS            64              // mais passos no MESMO ciclo
static const uint32_t CYCLE_PERIOD_MS = 150000; // 10*15000 original
#define HEAT_STABILIZE     600            // ↓ reduza se precisar fechar a janela

// Janela por passo (mantém ciclo igual)
static const uint32_t STEP_WINDOW_MS = CYCLE_PERIOD_MS / N_STEPS;

// 🌐 WiFi + MQTT
const char* WIFI_SSID   = "Quiet House";
const char* WIFI_PASS   = "quiethouse2025@";
const char* MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT= 1883;
const char* MQTT_TOPIC  = "application/test/caio/bme688";

// IDs
const uint16_t DEVICE_ID   = 1;
const uint16_t LOCATION_ID = 1;

// ===================== HEATER PROFILES (base 10) =====================
static const uint16_t tempBase[N_KIT_SENS][10] = {
  { 100,320,170,320,240,240,240,320,320,320 },
  { 100,320,170,320,240,240,240,320,320,320 },
  {  70,350,163,350,256,256,256,350,350,350 },
  {  70,350,163,350,256,256,256,350,350,350 },
  { 210,265,265,320,320,265,210,155,100,155 },
  { 210,265,265,320,320,265,210,155,100,155 },
  { 210,280,280,350,350,280,210,140, 70,140 },
  { 210,280,280,350,350,280,210,140, 70,140 },
};

static const uint16_t durBase[N_KIT_SENS][10] = {
  {  6020,  6300, 12320, 12600, 12880, 15680, 18620, 18900, 21700, 24640 },
  {  8960,  9240, 18200, 18480, 18760, 23100, 27580, 27860, 32200, 36680 },
  {  6020,  6300, 12320, 12600, 12880, 15680, 18620, 18900, 21700, 24640 },
  {  8960,  9240, 18200, 18480, 18760, 23100, 27580, 27860, 32200, 36680 },
  {  3360,  3640,  6720,  7000, 10080, 13440, 16800, 20160, 23520, 26880 },
  {  4480,  4760,  8960,  9240, 13440, 17920, 22400, 26880, 31360, 35840 },
  {  3360,  3640,  6720,  7000, 10080, 13440, 16800, 20160, 23520, 26880 },
  {  4480,  4760,  8960,  9240, 13440, 17920, 22400, 26880, 31360, 35840 }
};

// Interpolados para N_STEPS
uint16_t tempProfiles[N_KIT_SENS][N_STEPS];
uint16_t durProfiles [N_KIT_SENS][N_STEPS];

static inline uint16_t lerpU16(uint16_t a, uint16_t b, float t) {
  float v = a + (b - a) * t;
  if (v < 0) v = 0;
  if (v > 65535.0f) v = 65535.0f;
  return (uint16_t)(v + 0.5f);
}

static void buildProfilesN() {
  for (uint8_t s = 0; s < N_KIT_SENS; s++) {
    for (uint16_t k = 0; k < N_STEPS; k++) {
      float pos = (float)k * 10.0f / (float)N_STEPS;  // [0,10)
      int   i   = (int)floorf(pos);
      float f   = pos - (float)i;
      int   j   = (i + 1) % 10;
      tempProfiles[s][k] = lerpU16(tempBase[s][i], tempBase[s][j], f);
      durProfiles [s][k] = lerpU16(durBase [s][i], durBase [s][j], f);
    }
  }
}

// ===================== GLOBALS =====================
Bme68x bme[N_KIT_SENS];
comm_mux commSetup[N_KIT_SENS];
bool sensorActive[N_KIT_SENS] = {false};
uint8_t currentStep = 0;

SemaphoreHandle_t i2cMutex;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===================== WIFI / MQTT =====================
static void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println(" ✓");
}
static void connectMQTT() {
  while (!mqtt.connected()) {
    if (mqtt.connect("ESP32_BME688_CLIENT")) {
      Serial.println("MQTT connected");
    } else {
      Serial.printf("MQTT fail rc=%d\n", mqtt.state());
      delay(2000);
    }
  }
}

// ===================== PROTOBUF → MQTT =====================
static void publishProtobuf(uint8_t sensor_idx, uint8_t stepUsed, const bme68xData &d, uint32_t timestamp) {
  cedri_SensorGpsReading proto = cedri_SensorGpsReading_init_zero;
  proto.device_id           = DEVICE_ID;
  proto.location_id         = LOCATION_ID;
  proto.sensor_id           = sensor_idx;
  proto.heater_profile      = static_cast<cedri_HeaterProfile>(sensor_idx);
  proto.measurement_step    = stepUsed;                     // << usa o passo REAL medido (prevStep)
  proto.temp_c              = d.temperature;
  proto.humidity_pct        = d.humidity;
  proto.pressure_hpa        = d.pressure;
  proto.gas_resistance_ohm  = d.gas_resistance;
  proto.gas_valid           = d.status & BME68X_GASM_VALID_MSK;
  proto.heat_stable         = d.status & BME68X_HEAT_STAB_MSK;
  proto.timestamp           = timestamp;

  // Metadados (presentes no teu .proto; consumidores podem ignorar)
  proto.total_steps         = N_STEPS;
  proto.step_window_ms      = STEP_WINDOW_MS;
  proto.cycle_period_ms     = CYCLE_PERIOD_MS;
  proto.cycle_index         = 0; // (opcional: incremente a cada volta completa, se quiser)
  proto.heater_setpoint_c   = tempProfiles[sensor_idx][stepUsed];
  proto.heater_duration_ms  = durProfiles [sensor_idx][stepUsed];

  uint8_t buf[128];
  pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&os, cedri_SensorGpsReading_fields, &proto)) {
    Serial.println("[ERROR] Protobuf encode failed");
    return;
  }

  size_t out_len = 0;
  size_t out_buf_len = 256;
  uint8_t out_buf[out_buf_len];
  int ret = mbedtls_base64_encode(out_buf, out_buf_len, &out_len, buf, os.bytes_written);
  if (ret != 0) { Serial.println("[ERROR] Base64 encode failed"); return; }

  String b64 = String((char*)out_buf, out_len);
  String json = "{\"data\":\"" + b64 + "\"}";
  mqtt.publish(MQTT_TOPIC, json.c_str());
}

// ===================== MEASUREMENT TASK =====================
static void measurementTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    const uint8_t prevStep = (currentStep + N_STEPS - 1) % N_STEPS; // << passo realmente medido no ciclo anterior
    const uint32_t now = millis();

    // 1) Publica resultados do passo anterior (prevStep)
    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
      if (!sensorActive[i]) continue;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500))) {
        if (bme[i].fetchData()) {
          bme68xData d; uint8_t left; int retries = 5;
          do {
            left = bme[i].getData(d);
            if (d.status & BME68X_NEW_DATA_MSK) {
              publishProtobuf(i, prevStep, d, now); // << envia prevStep
            }
          } while (left && --retries > 0);
        }
        xSemaphoreGive(i2cMutex);
      }
    }

    // 2) Avança o passo alvo
    currentStep = (currentStep + 1) % N_STEPS;

    // 3) Programa TODOS os sensores para o novo passo e espera UMA vez o maior tempo
    uint32_t max_us = 0;
    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
      if (!sensorActive[i]) continue;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500))) {
        bme[i].setHeaterProf(tempProfiles[i][currentStep], durProfiles[i][currentStep]);
        bme[i].setOpMode(BME68X_FORCED_MODE);
        uint32_t us = bme[i].getMeasDur(BME68X_FORCED_MODE);
        if (us > max_us) max_us = us;
        xSemaphoreGive(i2cMutex);
      }
    }
    // Espera única pelo maior tempo de medição
    vTaskDelay(pdMS_TO_TICKS((max_us + 999) / 1000));

    // 4) Estabilização pequena para fechar janela (ajuste fino conforme necessário)
    vTaskDelay(pdMS_TO_TICKS(HEAT_STABILIZE));

    // 5) Mantém a periodicidade do passo (mesmo ciclo total)
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEP_WINDOW_MS));
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("=== BME688 + MQTT Node (fixed cycle, more steps) ===");
  Serial.printf("Cycle: %lu ms | Steps: %u | Step window: %lu ms\n",
                (unsigned long)CYCLE_PERIOD_MS, (unsigned)N_STEPS, (unsigned long)STEP_WINDOW_MS);

  buildProfilesN(); // gera 64 pontos a partir dos 10 de base

  i2cMutex = xSemaphoreCreateMutex();

  connectWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  Wire.begin(23, 22);
  Wire.setClock(400000);
  SPI.begin();
  comm_mux_begin(Wire, SPI);

  Serial.println("Initializing BME688 sensors...");
  for (uint8_t i = 0; i < N_KIT_SENS; i++) {
    commSetup[i] = comm_mux_set_config(Wire, SPI, i, commSetup[i]);
    bme[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &commSetup[i]);
    if (bme[i].checkStatus() == 0) {
      bme[i].setTPH();
      bme[i].setOpMode(BME68X_FORCED_MODE);
      sensorActive[i] = true;
      Serial.printf("✓ Sensor %d ready\n", i);
    } else {
      Serial.printf("✗ Sensor %d failed\n", i);
    }
  }

  xTaskCreatePinnedToCore(measurementTask, "Measure", 6144, NULL, 2, NULL, 1);
  Serial.println("=== Setup Complete ===");
}

void loop() {
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();
  delay(50);
}
