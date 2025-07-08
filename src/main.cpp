#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "lorae5.h"
#include "bme68xLibrary.h"
#include <commMux.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "pb_encode.h"
#include "measurement.pb.h"

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_VL53L0X.h>

// ===== CONFIGURATION =====
#define N_KIT_SENS        8
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

const String devEUI  = "0011223344556677";
const String appEUI  = "e30b08a3c0764c37";
const String appKey  = "00112233445566778899aabbccddeeff";
const String devAddr = "", nwkSKey = "", appSKey = "";

#define GPS_REQUIRED         false
#define GPS_PLACEHOLDER_LAT  0.0f
#define GPS_PLACEHOLDER_LON  0.0f
#define GPS_PLACEHOLDER_SATS 0

const uint16_t DEVICE_ID   = 1;
const uint16_t LOCATION_ID = 1;

// ===== SENSOR PROFILES =====
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

// ===== OBJECTS =====
Bme68x           bme[N_KIT_SENS];
comm_mux         commSetup[N_KIT_SENS];
bool             sensorActive[N_KIT_SENS] = {false};
uint8_t          currentStep = 0;

HardwareSerial   SerialGPS(2);
TinyGPSPlus      gps;

float            shared_lat = 0.0f, shared_lon = 0.0f;
uint8_t          shared_sats = 0;
SemaphoreHandle_t gpsMutex;

LORAE5 lorae5(devEUI, appEUI, appKey, devAddr, nwkSKey, appSKey);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ===== UTILS =====
float readToFVolume() {
    const float raio_cm = RESERVOIR_RADIUS_CM;
    const float altura_total_m = RESERVOIR_HEIGHT_M;
    const float pi = 3.14159265f;
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    float distancia_m = 0.0f;
    float volume_l = -1.0f;
    if (measure.RangeStatus != 4) {
        distancia_m = measure.RangeMilliMeter / 1000.0f;
        float altura_liquido = altura_total_m - distancia_m;
        if (altura_liquido < 0.0f) altura_liquido = 0.0f;
        if (altura_liquido > altura_total_m) altura_liquido = altura_total_m;
        float raio_m = raio_cm / 100.0f;
        volume_l = pi * raio_m * raio_m * altura_liquido * 1000.0f;
    } else {
        volume_l = -1.0f;
    }
    return volume_l;
}

void sendSensorGpsReading(
    uint8_t sensor_idx,
    uint8_t measurement_step,
    float   temp_c,
    float   humidity_pct,
    float   pressure_hpa,
    uint32_t gas_resistance_ohm,
    bool    gas_valid,
    bool    heat_stable,
    uint32_t timestamp_ms,
    float   latitude,
    float   longitude,
    uint32_t satellites,
    float   volume_l
) {
    cedri_SensorGpsReading proto = cedri_SensorGpsReading_init_zero;
    proto.device_id       = DEVICE_ID;
    proto.location_id     = LOCATION_ID;
    proto.sensor_id       = sensor_idx;
    proto.heater_profile  = static_cast<cedri_HeaterProfile>(sensor_idx);
    proto.measurement_step= measurement_step;
    proto.temp_c          = temp_c;
    proto.humidity_pct    = humidity_pct;
    proto.pressure_hpa    = pressure_hpa;
    proto.gas_resistance_ohm = gas_resistance_ohm;
    proto.gas_valid       = gas_valid;
    proto.heat_stable     = heat_stable;
    proto.timestamp       = timestamp_ms;
    proto.latitude        = latitude;
    proto.longitude       = longitude;
    proto.volume_l        = volume_l;
    uint8_t buffer[96];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, cedri_SensorGpsReading_fields, &proto)) {
        lorae5.sendData(buffer, stream.bytes_written);
        Serial.printf("[PAYLOAD] %zu bytes sent\n", stream.bytes_written);
    }
}

// ===== TASKS =====

void gpsTask(void*) {
    while (true) {
        while (SerialGPS.available()) gps.encode(SerialGPS.read());
        if (gps.location.isValid()) {
            if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
                shared_lat  = gps.location.lat();
                shared_lon  = gps.location.lng();
                shared_sats = gps.satellites.value();
                xSemaphoreGive(gpsMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void measurementTask(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    float lat, lon;
    uint8_t sats;
    while (true) {
        uint32_t now = millis();
        lat = GPS_PLACEHOLDER_LAT;
        lon = GPS_PLACEHOLDER_LON;
        sats = GPS_PLACEHOLDER_SATS;
        if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
            lat  = shared_lat;
            lon  = shared_lon;
            sats = shared_sats;
            xSemaphoreGive(gpsMutex);
        }

        float volume_l = readToFVolume();

        if (GPS_REQUIRED) {
            if (sats < 3 || (lat == 0.0f && lon == 0.0f)) {
                currentStep = (currentStep + 1) % MAX_MEASUREMENTS;
                vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEP_DURATION));
                continue;
            }
        }

        for (uint8_t i = 0; i < N_KIT_SENS; i++) {
            if (!sensorActive[i]) continue;
            if (bme[i].fetchData()) {
                bme68xData data;
                uint8_t left;
                do {
                    left = bme[i].getData(data);
                    if (data.status & BME68X_NEW_DATA_MSK) {
                        sendSensorGpsReading(
                            i, currentStep,
                            data.temperature,
                            data.humidity,
                            data.pressure,
                            data.gas_resistance,
                            data.status & BME68X_GASM_VALID_MSK,
                            data.status & BME68X_HEAT_STAB_MSK,
                            now,
                            lat, lon, sats,
                            volume_l
                        );
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                } while (left);
            }
        }

        currentStep = (currentStep + 1) % MAX_MEASUREMENTS;
        for (uint8_t i = 0; i < N_KIT_SENS; i++) {
            if (sensorActive[i]) {
                uint16_t t = tempProfiles[i][currentStep];
                uint16_t d = durProfiles[i][currentStep];
                bme[i].setHeaterProf(t, d);
                bme[i].setOpMode(BME68X_FORCED_MODE);
                uint32_t us = bme[i].getMeasDur(BME68X_FORCED_MODE);
                vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(HEAT_STABILIZE));
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEP_DURATION));
    }
}

// Extra task: periodic self-test
void selfTestTask(void*) {
    while (true) {
        bool allSensors = true;
        for (uint8_t i = 0; i < N_KIT_SENS; i++) {
            if (!sensorActive[i]) allSensors = false;
        }
        if (!lox.begin()) {
            Serial.println("[SELFTEST] VL53L0X failed.");
        }
        if (!allSensors) {
            Serial.println("[SELFTEST] One or more BME688 sensors failed.");
        }
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    while (!Serial) vTaskDelay(10);
    gpsMutex = xSemaphoreCreateMutex();

    Serial1.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    vTaskDelay(100);
    lorae5.setup_hardware(&Serial, &Serial1);
    lorae5.printInfo();

    lorae5.setup_lorawan(EU868, true, CLASS_A, 7, false, false, 8, false, 10000);
    Serial.print("[S] Joining LoRaWAN...");
    while (!lorae5.join()) {
        Serial.print(".");
        vTaskDelay(2000);
    }
    Serial.println("\n✓ Joined LoRaWAN");

    Wire.begin(23, 22);
    Wire.setClock(400000);
    SPI.begin();
    comm_mux_begin(Wire, SPI);

    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
        commSetup[i] = comm_mux_set_config(Wire, SPI, i, commSetup[i]);
        bme[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &commSetup[i]);
        if (bme[i].checkStatus() == 0) {
            bme[i].setTPH();
            bme[i].setOpMode(BME68X_FORCED_MODE);
            sensorActive[i] = true;
        }
    }

    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        while(1);
    }
    Serial.println(F("VL53L0X API successfully started."));

    xTaskCreatePinnedToCore(measurementTask, "Measure", 12288, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(selfTestTask, "SelfTest", 2048, NULL, 1, NULL, 1);
}

// ===== LOOP =====
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}