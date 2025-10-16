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
#define N_KIT_SENS            8
#define N_STEPS               16                          // <— 16 steps
static const uint32_t CYCLE_PERIOD_MS = 150000;           // 150 s total
static const uint32_t STEP_WINDOW_MS  = CYCLE_PERIOD_MS / N_STEPS;
#define HEAT_STABILIZE_MS     700                         // estabilização pós-medida (ajuste fino)

// 🌐 WiFi + MQTT
const char* WIFI_SSID   = "Quiet House1";                  // <<< confirme!
const char* WIFI_PASS   = "quiethouse2025@";
const char* MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT= 1883;
const char* MQTT_TOPIC  = "application/test/caio/bme688";

// IDs
const uint16_t DEVICE_ID   = 1;
const uint16_t LOCATION_ID = 1;

// ===================== HEATER PROFILES (10 pontos do PDF) =====================
// temperaturas dos 10 “control points” por perfil (HP-411..HP-504)
static const uint16_t tempBase[N_KIT_SENS][10] = {
  { 100,320,170,320,240,240,240,320,320,320 },  // HP-411
  { 100,320,170,320,240,240,240,320,320,320 },  // HP-412
  {  70,350,163,350,256,256,256,350,350,350 },  // HP-413
  {  70,350,163,350,256,256,256,350,350,350 },  // HP-414
  { 210,265,265,320,320,265,210,155,100,155 },  // HP-501
  { 210,265,265,320,320,265,210,155,100,155 },  // HP-502
  { 210,280,280,350,350,280,210,140, 70,140 },  // HP-503
  { 210,280,280,350,350,280,210,140, 70,140 }   // HP-504
};

// marcos ABSOLUTOS (ms) no fim de cada ponto base (do PDF, cumulativos)
static const uint32_t durBaseAbs[N_KIT_SENS][10] = {
  {  6020,  6300, 12320, 12600, 12880, 15680, 18620, 18900, 21700, 24640 }, // HP-411
  {  8960,  9240, 18200, 18480, 18760, 23100, 27580, 27860, 32200, 36680 }, // HP-412
  {  6020,  6300, 12320, 12600, 12880, 15680, 18620, 18900, 21700, 24640 }, // HP-413
  {  8960,  9240, 18200, 18480, 18760, 23100, 27580, 27860, 32200, 36680 }, // HP-414
  {  3360,  3640,  6720,  7000, 10080, 13440, 16800, 20160, 23520, 26880 }, // HP-501
  {  4480,  4760,  8960,  9240, 13440, 17920, 22400, 26880, 31360, 35840 }, // HP-502
  {  3360,  3640,  6720,  7000, 10080, 13440, 16800, 20160, 23520, 26880 }, // HP-503
  {  4480,  4760,  8960,  9240, 13440, 17920, 22400, 26880, 31360, 35840 }  // HP-504
};

// perfis upsampled (16 pontos)
uint16_t tempProfiles[N_KIT_SENS][N_STEPS];
uint16_t durProfiles [N_KIT_SENS][N_STEPS];

uint32_t cycleIndex = 0;

// ===================== GLOBALS =====================
Bme68x bme[N_KIT_SENS];
comm_mux commSetup[N_KIT_SENS];
bool sensorActive[N_KIT_SENS] = {false};
uint8_t currentStep = 0;
SemaphoreHandle_t busMutex;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===================== HELPERS =====================
static inline uint16_t lerpU16(uint16_t a, uint16_t b, float t) {
  float v = a + (b - a) * t;
  if (v < 0) v = 0;
  if (v > 65535.0f) v = 65535.0f;
  return (uint16_t)(v + 0.5f);
}

// Gera 16 steps preservando o “shape” do PDF.
// Temperatura: interpola 10→16. Duração: interpola marcos absolutos 0..10 → 17 bordas e tira os deltas.
// Não estico artificialmente — só limito mínimo e deixo slack na janela (reduz artefatos).
static void buildProfilesN() {
  const uint32_t overheadMs   = 1000; // margem para publish e respiros
  const uint32_t heaterBudget = (STEP_WINDOW_MS > (HEAT_STABILIZE_MS + overheadMs))
                              ? (STEP_WINDOW_MS - HEAT_STABILIZE_MS - overheadMs)
                              : (STEP_WINDOW_MS / 2);

  uint32_t maxDelta = 0, minDelta = 0xFFFFFFFF;

  for (uint8_t s = 0; s < N_KIT_SENS; s++) {
    // A) temperatura
    for (uint16_t k = 0; k < N_STEPS; k++) {
      float pos = (float)k * 9.0f / (float)(N_STEPS - 1); // 0..9 inclusive
      int   i   = (int)floorf(pos);
      float f   = pos - (float)i;
      if (i >= 9) { i = 9; f = 0.0f; }
      tempProfiles[s][k] = lerpU16(tempBase[s][i], tempBase[s][i+1], f);
    }

    // B) deltas de tempo a partir dos marcos absolutos
    uint32_t baseMarks[11]; baseMarks[0] = 0;
    for (int i = 0; i < 10; i++) baseMarks[i+1] = durBaseAbs[s][i];

    uint32_t marks[N_STEPS + 1];
    for (uint16_t k = 0; k <= N_STEPS; k++) {
      float pos = (float)k * 10.0f / (float)N_STEPS; // 0..10
      int   i   = (int)floorf(pos);
      float f   = pos - (float)i;
      if (i >= 10) { i = 10; f = 0.0f; }
      int j = (i < 10) ? (i + 1) : 10;
      float a = (float)baseMarks[i], b = (float)baseMarks[j];
      marks[k] = (uint32_t)(a + (b - a) * f + 0.5f);
    }

    for (uint16_t k = 0; k < N_STEPS; k++) {
      uint32_t delta = (marks[k+1] > marks[k]) ? (marks[k+1] - marks[k]) : 1;
      // clamp suave: ≥ 200 ms e ≤ orçamento do step (temos slack de janela grande)
      if (delta < 200) delta = 200;
      if (delta > heaterBudget) delta = heaterBudget;
      durProfiles[s][k] = (uint16_t)delta;
      if (delta > maxDelta) maxDelta = delta;
      if (delta < minDelta) minDelta = delta;
    }
  }

  Serial.printf("[Profiles] steps=%u | stepWindow=%lu ms | heaterBudget=%lu ms | dur[min..max]=%lu..%lu ms\n",
                (unsigned)N_STEPS, (unsigned long)STEP_WINDOW_MS, (unsigned long)heaterBudget,
                (unsigned long)minDelta, (unsigned long)maxDelta);
}

static void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  for (int i=0; i<40 && WiFi.status()!=WL_CONNECTED; ++i) { delay(500); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf(" ✓  IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println(" FAILED (offline mode)");
  }
}

static void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  while (!mqtt.connected()) {
    String cid = "ESP32_BME688_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str())) {
      Serial.println("MQTT connected");
    } else {
      Serial.printf("MQTT fail rc=%d\n", mqtt.state());
      delay(1000);
    }
  }
}

// ===================== MQTT / PROTOBUF =====================
static bool publishProtobuf(uint8_t sensor_idx, uint8_t stepUsed, const bme68xData &d, uint32_t timestamp) {
  if (WiFi.status() != WL_CONNECTED || !mqtt.connected()) return false;

  cedri_SensorGpsReading proto = cedri_SensorGpsReading_init_zero;
  proto.device_id           = DEVICE_ID;
  proto.location_id         = LOCATION_ID;
  proto.sensor_id           = sensor_idx;
  proto.heater_profile      = static_cast<cedri_HeaterProfile>(sensor_idx);
  proto.measurement_step    = stepUsed;

  // leituras
  proto.temp_c              = d.temperature;
  proto.humidity_pct        = d.humidity;
  proto.pressure_hpa        = d.pressure;
  proto.gas_resistance_ohm  = d.gas_resistance;
  proto.gas_valid           = d.status & BME68X_GASM_VALID_MSK;
  proto.heat_stable         = d.status & BME68X_HEAT_STAB_MSK;
  proto.timestamp           = timestamp;

  // metadados do ciclo + setpoints do step
  proto.total_steps         = N_STEPS;
  proto.step_window_ms      = STEP_WINDOW_MS;
  proto.cycle_period_ms     = CYCLE_PERIOD_MS;
  proto.cycle_index         = cycleIndex;
  proto.heater_setpoint_c   = tempProfiles[sensor_idx][stepUsed];
  proto.heater_duration_ms  = durProfiles [sensor_idx][stepUsed];

  uint8_t buf[512];
  pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&os, cedri_SensorGpsReading_fields, &proto)) {
    Serial.println("[PB] encode failed");
    return false;
  }

  size_t out_len = 0;
  static uint8_t out_buf[800]; // ~4/3 do payload + folga
  if (mbedtls_base64_encode(out_buf, sizeof(out_buf), &out_len, buf, os.bytes_written) != 0) {
    Serial.println("[PB] b64 encode failed");
    return false;
  }

  String json = "{\"data\":\"" + String((char*)out_buf, out_len) + "\"}";
  bool ok = mqtt.publish(MQTT_TOPIC, json.c_str(), false);
  if (!ok) Serial.printf("[MQTT] publish failed (len=%u, state=%d)\n", (unsigned)json.length(), mqtt.state());
  return ok;
}

// ===================== MEASUREMENT TASK =====================
// Estratégia (FORCED_MODE):
// 1) Para o step k: programa todos os sensores (set heater + FORCED).
// 2) Espera o maior entre (heater dur) e (meas dur).
// 3) Dá uma estabilização curta.
// 4) Poll não-bloqueante de NEW_DATA por sensor (máx. 300 ms), publica se houver.
// 5) Mantém a janela do step (9375 ms) com slack grande para evitar “empilhar” leituras.
static void measurementTask(void*) {
  vTaskDelay(pdMS_TO_TICKS(1200));

  for (;;) {
    const uint8_t step = currentStep;
    const uint32_t stepT0 = millis();
    uint32_t maxWaitMs = 0;

    // 1) programa todos
    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
      if (!sensorActive[i]) continue;

      if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(400))) {
        const uint16_t setTemp = tempProfiles[i][step];
        const uint16_t setDur  = durProfiles [i][step];

        if (i == 0) {
          Serial.printf("[STEP %u] T=%uC dur=%ums\n", step, setTemp, setDur);
        }

        bme[i].setHeaterProf(setTemp, setDur);
        bme[i].setOpMode(BME68X_FORCED_MODE);

        const uint32_t measMs  = (bme[i].getMeasDur(BME68X_FORCED_MODE) + 999) / 1000;
        const uint32_t wait    = (measMs > setDur ? measMs : setDur);
        if (wait > maxWaitMs) maxWaitMs = wait;

        xSemaphoreGive(busMutex);
      }
    }

    // 2) espera o necessário + 3) estabiliza
    vTaskDelay(pdMS_TO_TICKS(maxWaitMs + HEAT_STABILIZE_MS));

    // 4) coleta/publica
    const uint32_t now = millis();
    uint8_t pubs = 0;

    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
      if (!sensorActive[i]) continue;

      if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(400))) {
        bme68xData d{};
        bool got = false;
        uint32_t t0 = millis();

        // curto polling para garantir NEW_DATA
        while (millis() - t0 < 300) {
          if (bme[i].fetchData()) {
            uint8_t left = bme[i].getData(d);
            if (d.status & BME68X_NEW_DATA_MSK) { got = true; break; }
            if (!left) break;
          }
          vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (got) {
          if (publishProtobuf(i, step, d, now)) pubs++;
        }
        xSemaphoreGive(busMutex);
      }
    }

    // contabilidade de ciclo
    currentStep = (currentStep + 1) % N_STEPS;
    if (currentStep == 0) cycleIndex++;

    // 5) respeita janela do step
    uint32_t elapsed = millis() - stepT0;
    if (elapsed < STEP_WINDOW_MS) vTaskDelay(pdMS_TO_TICKS(STEP_WINDOW_MS - elapsed));

    if (pubs == 0) {
      // ajuda no diagnóstico quando nada foi publicado no step
      // Serial.printf("[STEP %u] no publish (timeout or no NEW_DATA)\n", step);
    }
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n=== BME688 + MQTT Node (16 steps, fixed cycle) ===");
  Serial.printf("Cycle=%lu ms | Steps=%u | StepWindow=%lu ms\n",
                (unsigned long)CYCLE_PERIOD_MS, (unsigned)N_STEPS, (unsigned long)STEP_WINDOW_MS);

  buildProfilesN();

  busMutex = xSemaphoreCreateMutex();

  connectWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(60);
  mqtt.setBufferSize(2048);
  if (WiFi.status() == WL_CONNECTED) connectMQTT();

  // Barramentos + MUX (ordem importa)
  Wire.begin(23, 22);                 // ajuste para seu hardware
  Wire.setClock(400000);
  SPI.begin();
  comm_mux_begin(Wire, SPI);

  Serial.println("Initializing BME688 sensors...");
  for (uint8_t i = 0; i < N_KIT_SENS; i++) {
    vTaskDelay(pdMS_TO_TICKS(50));
    commSetup[i] = comm_mux_set_config(Wire, SPI, i, commSetup[i]);
    bme[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &commSetup[i]);

    if (bme[i].checkStatus() == 0) {
      // oversampling estáveis (evita latência exagerada)
      bme[i].setTPH(BME68X_OS_2X, BME68X_OS_16X, BME68X_OS_1X);
      bme[i].setOpMode(BME68X_FORCED_MODE);
      sensorActive[i] = true;
      Serial.printf("✓ Sensor %u ready\n", i);
    } else {
      sensorActive[i] = false;
      Serial.printf("✗ Sensor %u failed: %s\n", i, bme[i].statusString());
    }
  }

  xTaskCreatePinnedToCore(measurementTask, "Measure", 8192, NULL, 2, NULL, 1);
  Serial.println("=== Setup Complete, starting measurement cycles... ===");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) connectMQTT();
    mqtt.loop();
  }
  delay(15);
}

