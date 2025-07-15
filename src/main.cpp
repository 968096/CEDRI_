#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <commMux.h>
#include "lorae5.h"
#include "bme68xLibrary.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "pb_encode.h"
#include "measurement.pb.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_VL53L0X.h>

// ===== DEBUG =====
#define DEBUG_LEVEL 1

#if DEBUG_LEVEL > 0
  #define DEBUG_PRINTF(...)   Serial.printf(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTLN(...)
#endif

// ===== CONFIGURATION =====
#define N_KIT_SENS         8
#define MAX_MEASUREMENTS  10
#define STEP_DURATION     5000
#define HEAT_STABILIZE    2000

#define LORA_RX_PIN       16
#define LORA_TX_PIN       17
#define LORA_BAUD         9600

#define GPS_RX_PIN        33
#define GPS_TX_PIN        32
#define GPS_BAUD          9600

#define RESERVOIR_RADIUS_CM 40.0f
#define RESERVOIR_HEIGHT_M  1.1f

#define GPS_REQUIRED         false
#define GPS_PLACEHOLDER_LAT  0.0f
#define GPS_PLACEHOLDER_LON  0.0f
#define GPS_PLACEHOLDER_SATS 0

const uint16_t DEVICE_ID   = 1;
const uint16_t LOCATION_ID = 1;

// ===== HEATER PROFILES =====
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

// ===== GLOBALS & OBJECTS =====
Bme68x           bme[N_KIT_SENS];
comm_mux         commSetup[N_KIT_SENS];
bool             sensorActive[N_KIT_SENS] = {false};
uint8_t          currentStep = 0;

HardwareSerial   SerialGPS(2);
TinyGPSPlus      gps;
float            shared_lat = 0.0f, shared_lon = 0.0f;
uint8_t          shared_sats = 0;
SemaphoreHandle_t gpsMutex;

SemaphoreHandle_t loraTxSemaphore;
// ===== MUTEX PARA BARRAMENTO COMPARTILHADO =====
SemaphoreHandle_t i2cMutex; // Protege BME688 e VL53L0X

LORAE5           lorae5(
  F("0011223344556677"),
  F("e30b08a3c0764c37"),
  F("00112233445566778899aabbccddeeff"),
  F(""), F(""), F("")
);

Adafruit_VL53L0X lox;

// ===== UTILITIES =====
float readToFVolume() {
    const float pi = 3.14159265f;
    VL53L0X_RangingMeasurementData_t m;
    // PROTEGE ACESSO AO BARRAMENTO
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000))) {
        lox.rangingTest(&m, false);
        xSemaphoreGive(i2cMutex);
    } else {
        DEBUG_PRINTLN("[ERROR] VL53L0X I2C bus lock timeout");
        return -1.0f;
    }
    if (m.RangeStatus == 4) return -1.0f;
    float dist = m.RangeMilliMeter / 1000.0f;           // m
    float h_liquid = RESERVOIR_HEIGHT_M - dist;
    h_liquid = constrain(h_liquid, 0.0f, RESERVOIR_HEIGHT_M);
    float r = RESERVOIR_RADIUS_CM / 100.0f;             // m
    return pi * r * r * h_liquid * 1000.0f;             // L
}

void sendSensorGpsReading(
    uint8_t sensor_idx, uint8_t measurement_step,
    float temp_c, float humidity_pct, float pressure_hpa,
    uint32_t gas_resistance_ohm, bool gas_valid, bool heat_stable,
    uint32_t timestamp_ms, float latitude, float longitude,
    uint32_t satellites, float volume_l
) {
    // 1) build protobuf
    cedri_SensorGpsReading proto = cedri_SensorGpsReading_init_zero;
    proto.device_id         = DEVICE_ID;
    proto.location_id       = LOCATION_ID;
    proto.sensor_id         = sensor_idx;
    proto.heater_profile    = static_cast<cedri_HeaterProfile>(sensor_idx);
    proto.measurement_step  = measurement_step;
    proto.temp_c            = temp_c;
    proto.humidity_pct      = humidity_pct;
    proto.pressure_hpa      = pressure_hpa;
    proto.gas_resistance_ohm= gas_resistance_ohm;
    proto.gas_valid         = gas_valid;
    proto.heat_stable       = heat_stable;
    proto.timestamp         = timestamp_ms;
    proto.latitude          = latitude;
    proto.longitude         = longitude;
    proto.volume_l          = volume_l;

    uint8_t buf[96];
    pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&os, cedri_SensorGpsReading_fields, &proto)) {
        Serial.println("[ERROR] Protobuf encode failed");
        return;
    }

    // 2) send uplink
    lorae5.sendData(buf, os.bytes_written);
    DEBUG_PRINTF("[PAYLOAD] %u bytes sent, waiting for TX…\n", os.bytes_written);

    // 3) wait on TX-done semaphore (from LoRa monitor task)
    if (xSemaphoreTake(loraTxSemaphore, pdMS_TO_TICKS(6000))) {
        DEBUG_PRINTLN("[LoRa] TX complete");
    } else {
        DEBUG_PRINTLN("[LoRa] WARNING: no TX complete seen");
    }
}

// ===== FREERTOS TASKS =====
void gpsTask(void*) {
    for (;;) {
        while (SerialGPS.available()) {
            gps.encode(SerialGPS.read());
        }
        if (gps.location.isValid()) {
            if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
                shared_lat  = gps.location.lat();
                shared_lon  = gps.location.lng();
                shared_sats = gps.satellites.value();
                xSemaphoreGive(gpsMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // lower poll rate
    }
}

void measurementTask(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        uint32_t now = millis();
        float lat = GPS_PLACEHOLDER_LAT, lon = GPS_PLACEHOLDER_LON;
        uint8_t sats = GPS_PLACEHOLDER_SATS;
        if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
            lat = shared_lat;
            lon = shared_lon;
            sats = shared_sats;
            xSemaphoreGive(gpsMutex);
        }

        float vol = readToFVolume();
        if (GPS_REQUIRED && (sats < 3 || (lat == 0 && lon == 0))) {
            currentStep = (currentStep + 1) % MAX_MEASUREMENTS;
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEP_DURATION));
            continue;
        }

        // PROTEGE ACESSO BME688 COM MUTEX
        for (uint8_t i = 0; i < N_KIT_SENS; i++) {
            if (!sensorActive[i]) continue;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000))) {
                if (bme[i].fetchData()) {
                    bme68xData d; uint8_t left;
                    do {
                        left = bme[i].getData(d);
                        if (d.status & BME68X_NEW_DATA_MSK) {
                            xSemaphoreGive(i2cMutex);
                            sendSensorGpsReading(
                              i, currentStep,
                              d.temperature, d.humidity, d.pressure,
                              d.gas_resistance,
                              d.status & BME68X_GASM_VALID_MSK,
                              d.status & BME68X_HEAT_STAB_MSK,
                              now, lat, lon, sats, vol
                            );
                            vTaskDelay(pdMS_TO_TICKS(50));
                            // re-tenta pegar mutex caso tenha mais dados
                            if (left && !xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000))) break;
                        }
                    } while (left);
                } else {
                    xSemaphoreGive(i2cMutex);
                }
            } else {
                DEBUG_PRINTLN("[ERROR] BME688 I2C bus lock timeout");
            }
        }

        currentStep = (currentStep + 1) % MAX_MEASUREMENTS;
        for (uint8_t i = 0; i < N_KIT_SENS; i++) {
            if (!sensorActive[i]) continue;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000))) {
                uint16_t t = tempProfiles[i][currentStep];
                uint16_t d = durProfiles[i][currentStep];
                bme[i].setHeaterProf(t,d);
                bme[i].setOpMode(BME68X_FORCED_MODE);
                uint32_t us = bme[i].getMeasDur(BME68X_FORCED_MODE);
                xSemaphoreGive(i2cMutex);
                vTaskDelay(pdMS_TO_TICKS((us+999)/1000));
            } else {
                DEBUG_PRINTLN("[ERROR] BME688 I2C bus lock timeout (heater)");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(HEAT_STABILIZE));
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEP_DURATION));
    }
}

void selfTestTask(void*) {
    for (;;) {
        bool allOK = true;
        for (uint8_t i = 0; i < N_KIT_SENS; i++) {
            if (!sensorActive[i]) allOK = false;
        }
        // PROTEGE TESTE DO VL53L0X COM MUTEX
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(3000))) {
            if (!lox.begin())       Serial.println("[SELFTEST] VL53L0X fail");
            xSemaphoreGive(i2cMutex);
        } else {
            Serial.println("[ERROR] VL53L0X I2C bus lock timeout (selftest)");
        }
        if (!allOK) Serial.println("[SELFTEST] BME688 fail");
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

void loraMonitorTask(void*) {
    char buf[64];
    size_t idx = 0;
    for (;;) {
        while (Serial1.available()) {
            char c = Serial1.read();
            if (c == '\r' || c == '\n') {
                if (idx > 0) {
                    buf[idx] = '\0';
                    if (strstr(buf, "Transmission Done")) {
                        xSemaphoreGive(loraTxSemaphore);
                    }
                    idx = 0;
                }
            } else if (idx < sizeof(buf) - 1) {
                buf[idx++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) vTaskDelay(pdMS_TO_TICKS(10));

    // Semaphores
    gpsMutex        = xSemaphoreCreateMutex();
    loraTxSemaphore = xSemaphoreCreateBinary();
    i2cMutex        = xSemaphoreCreateMutex(); // MUTEX barramento

    // LoRa E5 on Serial1
    Serial1.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));
    lorae5.setup_hardware(&Serial, &Serial1);
    lorae5.printInfo();
    lorae5.setup_lorawan(EU868, true, CLASS_A, 7, false, false, 8, false, 10000);

    Serial.print("[S] Joining LoRaWAN…");
    while (!lorae5.join()) {
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    Serial.println("\n✓ Joined LoRaWAN");

    // BME688 + commMux
    Wire.begin(23,22); Wire.setClock(400000);
    SPI.begin();
    comm_mux_begin(Wire,SPI);
    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
        commSetup[i] = comm_mux_set_config(Wire, SPI, i, commSetup[i]);
        bme[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &commSetup[i]);
        if (bme[i].checkStatus() == 0) {
            bme[i].setTPH();
            bme[i].setOpMode(BME68X_FORCED_MODE);
            sensorActive[i] = true;
        }
    }

    // GPS
    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    // VL53L0X
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000))) {
        if (!lox.begin()) {
            Serial.println("Failed to boot VL53L0X");
            while (1);
        }
        xSemaphoreGive(i2cMutex);
    }
    Serial.println("VL53L0X API successfully started.");

    // Start tasks
    xTaskCreatePinnedToCore(measurementTask, "Measure", 6144, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(gpsTask,        "GPSTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(selfTestTask,   "SelfTest",2048, NULL, 0, NULL, 1);
    xTaskCreatePinnedToCore(loraMonitorTask,"LoRaMon", 4096, NULL, 3, NULL, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}