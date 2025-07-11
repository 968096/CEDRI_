#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <cstdarg>
#include "lorae5.h"
#include "bme68xLibrary.h"
#include <commMux.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "pb_encode.h"
#include "measurement.pb.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "Adafruit_VL53L0X.h"

// ===== CONFIGURATION =====
#define N_KIT_SENS        8
#define MAX_MEASUREMENTS  10
#define STEP_DURATION     10000
#define HEAT_STABILIZE    2000

#define LORA_RX_PIN       16
#define LORA_TX_PIN       17
#define LORA_BAUD         9600

#define GPS_RX_PIN        33
#define GPS_TX_PIN        32
#define GPS_BAUD          9600

#define RESERVOIR_RADIUS_CM 40.0f
#define RESERVOIR_HEIGHT_M  1.1f

#define MUTEX_TIMEOUT_MS    1000
#define MUTEX_TIMEOUT_TICKS pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)
#define LORA_QUEUE_LEN      32

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

SemaphoreHandle_t spiMutex;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t sensorStateMutex;

QueueHandle_t    loraUplinkQueue;

LORAE5 lorae5("0011223344556677", "e30b08a3c0764c37", "00112233445566778899aabbccddeeff", "", "", "");
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

struct LoraUplinkMsg {
  uint8_t data[96];
  size_t len;
};

// ===== UTILS =====

void safePrint(const char* message) {
    if (xSemaphoreTake(serialMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        Serial.print(message);
        xSemaphoreGive(serialMutex);
    }
}
void safePrintln(const char* message) {
    if (xSemaphoreTake(serialMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        Serial.println(message);
        xSemaphoreGive(serialMutex);
    }
}
void safePrintf(const char* format, ...) {
    if (xSemaphoreTake(serialMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        Serial.print(buffer);
        xSemaphoreGive(serialMutex);
    }
}

// Thread-safe helpers
bool getSensorState(uint8_t idx) {
    bool state = false;
    if (xSemaphoreTake(sensorStateMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        state = sensorActive[idx];
        xSemaphoreGive(sensorStateMutex);
    }
    return state;
}
void setSensorState(uint8_t idx, bool state) {
    if (xSemaphoreTake(sensorStateMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        sensorActive[idx] = state;
        xSemaphoreGive(sensorStateMutex);
    }
}
uint8_t getCurrentStep() {
    uint8_t step = 0;
    if (xSemaphoreTake(sensorStateMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        step = currentStep;
        xSemaphoreGive(sensorStateMutex);
    }
    return step;
}
void incrementCurrentStep() {
    if (xSemaphoreTake(sensorStateMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        currentStep = (currentStep + 1) % MAX_MEASUREMENTS;
        xSemaphoreGive(sensorStateMutex);
    }
}

// Read ToF sensor and calculate liquid volume
float readToFVolume() {
    const float pi = 3.14159265f;
    float volume_l = -1.0f;
    if (xSemaphoreTake(i2cMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        VL53L0X_RangingMeasurementData_t measure;
        lox.rangingTest(&measure, false);
        if (measure.RangeStatus != 4) {
            float dist = measure.RangeMilliMeter / 1000.0f;
            float h_liquid = RESERVOIR_HEIGHT_M - dist;
            h_liquid = constrain(h_liquid, 0.0f, RESERVOIR_HEIGHT_M);
            float r = RESERVOIR_RADIUS_CM / 100.0f;
            volume_l = pi * r * r * h_liquid * 1000.0f;
        }
        xSemaphoreGive(i2cMutex);
    }
    return volume_l;
}

void enqueueSensorGpsReading(
    uint8_t sensor_idx, uint8_t measurement_step,
    float temp_c, float humidity_pct, float pressure_hpa,
    uint32_t gas_resistance_ohm, bool gas_valid, bool heat_stable,
    uint32_t timestamp_ms, float latitude, float longitude,
    uint32_t satellites, float volume_l
) {
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
        safePrintln("[ERROR] Protobuf encode failed");
        return;
    }
    LoraUplinkMsg msg;
    memcpy(msg.data, buf, os.bytes_written);
    msg.len = os.bytes_written;

    // Espera espaço na fila se estiver cheia (rate limit)
    while (uxQueueSpacesAvailable(loraUplinkQueue) == 0) {
        safePrintln("[WARN] LoRa uplink queue full, aguardando espaço...");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (xQueueSend(loraUplinkQueue, &msg, 0) != pdTRUE) {
        safePrintln("[ERROR] Falha ao inserir na fila LoRa uplink!");
    }
}

// ===== TASKS =====

void gpsTask(void*) {
    while (true) {
        while (SerialGPS.available()) gps.encode(SerialGPS.read());
        if (gps.location.isValid()) {
            if (xSemaphoreTake(gpsMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
                shared_lat = gps.location.lat();
                shared_lon = gps.location.lng();
                shared_sats = gps.satellites.value();
                xSemaphoreGive(gpsMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void measurementTask(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        uint32_t now = millis();
        float lat = GPS_PLACEHOLDER_LAT, lon = GPS_PLACEHOLDER_LON;
        uint8_t sats = GPS_PLACEHOLDER_SATS;
        if (xSemaphoreTake(gpsMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
            lat = shared_lat;
            lon = shared_lon;
            sats = shared_sats;
            xSemaphoreGive(gpsMutex);
        }

        float vol = readToFVolume();
        uint8_t currentMeasurementStep = getCurrentStep();

        if (GPS_REQUIRED && (sats < 3 || (lat == 0 && lon == 0))) {
            incrementCurrentStep();
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEP_DURATION));
            continue;
        }

        for (uint8_t i = 0; i < N_KIT_SENS; i++) {
            if (!getSensorState(i)) continue;
            if (bme[i].fetchData()) {
                bme68xData d; uint8_t left;
                do {
                    left = bme[i].getData(d);
                    if (d.status & BME68X_NEW_DATA_MSK) {
                        enqueueSensorGpsReading(
                            i, currentMeasurementStep,
                            d.temperature, d.humidity, d.pressure,
                            d.gas_resistance,
                            d.status & BME68X_GASM_VALID_MSK,
                            d.status & BME68X_HEAT_STAB_MSK,
                            now, lat, lon, sats, vol
                        );
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                } while (left);
            }
        }

        incrementCurrentStep();
        currentMeasurementStep = getCurrentStep();

        for (uint8_t i = 0; i < N_KIT_SENS; i++) {
            if (!getSensorState(i)) continue;
            uint16_t t = tempProfiles[i][currentMeasurementStep];
            uint16_t d = durProfiles[i][currentMeasurementStep];
            bme[i].setHeaterProf(t,d);
            bme[i].setOpMode(BME68X_FORCED_MODE);
            uint32_t us = bme[i].getMeasDur(BME68X_FORCED_MODE);
            vTaskDelay(pdMS_TO_TICKS((us+999)/1000));
        }

        vTaskDelay(pdMS_TO_TICKS(HEAT_STABILIZE));
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEP_DURATION));
    }
}

void loraUplinkTask(void*) {
    LoraUplinkMsg msg;
    for (;;) {
        if (xQueueReceive(loraUplinkQueue, &msg, portMAX_DELAY) == pdTRUE) {
            lorae5.sendData(msg.data, msg.len);
            safePrintf("[LoRaUplink] Sent %d bytes\n", msg.len);
        }
    }
}

void selfTestTask(void*) {
    for (;;) {
        bool allSensors = true;
        for (uint8_t i = 0; i < N_KIT_SENS; i++) if (!getSensorState(i)) allSensors = false;
        if (xSemaphoreTake(i2cMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
            if (!lox.begin()) safePrintln("[SELFTEST] VL53L0X fail");
            xSemaphoreGive(i2cMutex);
        }
        if (!allSensors) safePrintln("[SELFTEST] BME688 fail");
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    while (!Serial) vTaskDelay(10);

    gpsMutex = xSemaphoreCreateMutex();
    spiMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();
    serialMutex = xSemaphoreCreateMutex();
    sensorStateMutex = xSemaphoreCreateMutex();
    loraUplinkQueue = xQueueCreate(LORA_QUEUE_LEN, sizeof(LoraUplinkMsg));

    if (!gpsMutex || !spiMutex || !i2cMutex || !serialMutex || !sensorStateMutex || !loraUplinkQueue) {
        Serial.println("[FATAL] Failed to create mutexes!");
        while(1) vTaskDelay(1000);
    }

    safePrintln("[INIT] All mutexes and queues created successfully");

    Serial1.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    vTaskDelay(100);

    lorae5.setup_hardware(&Serial, &Serial1);
    lorae5.printInfo();
    lorae5.setup_lorawan(EU868, true, CLASS_A, 7, false, false, 8, false, 10000);

    safePrint("[S] Joining LoRaWAN...");
    bool joined = false;
    while (!joined) {
        joined = lorae5.join();
        if (!joined) {
            safePrint(".");
            vTaskDelay(2000);
        }
    }
    safePrintln("\n✓ Joined LoRaWAN");

    if (xSemaphoreTake(i2cMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        if (xSemaphoreTake(spiMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
            Wire.begin(23, 22);
            Wire.setClock(400000);
            SPI.begin();
            comm_mux_begin(Wire, SPI);
            xSemaphoreGive(spiMutex);
        }
        xSemaphoreGive(i2cMutex);
    }

    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
        commSetup[i] = comm_mux_set_config(Wire, SPI, i, commSetup[i]);
        bme[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &commSetup[i]);
        if (bme[i].checkStatus() == 0) {
            bme[i].setTPH();
            bme[i].setOpMode(BME68X_FORCED_MODE);
            setSensorState(i, true);
        }
    }

    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    if (xSemaphoreTake(i2cMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
        if (!lox.begin()) {
            safePrintln("Failed to boot VL53L0X");
            while(1) vTaskDelay(1000);
        }
        xSemaphoreGive(i2cMutex);
    }
    safePrintln("VL53L0X API successfully started.");

    xTaskCreatePinnedToCore(measurementTask, "Measure", 16384, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(selfTestTask, "SelfTest", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(loraUplinkTask, "LoRaUp", 4096, NULL, 2, NULL, 1);

    safePrintln("[INIT] All tasks created successfully");
}

// ===== LOOP =====
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}