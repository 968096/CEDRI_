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

// GPS libraries
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ========== CONFIGURATION ==========

// Number of BME68x sensors and measurement steps
#define N_KIT_SENS       8
#define MAX_MEASUREMENTS 10

// Timing for sensor routine
#define STEP_DURATION    5000  // ms between each measurement step
#define HEAT_STABILIZE   2000  // ms to wait after heater change

// LoRa module pins and baudrate (Serial1)
#define LORA_RX_PIN    16
#define LORA_TX_PIN    17
#define LORA_BAUD      9600

// GPS module pins and baudrate (HardwareSerial 2)
#define GPS_RX_PIN     33  // Adjust to your wiring
#define GPS_TX_PIN     32
#define GPS_BAUD       9600

const String devEUI  = "0011223344556677";
const String appEUI  = "e30b08a3c0764c37";
const String appKey  = "00112233445566778899aabbccddeeff";
const String devAddr = "", nwkSKey = "", appSKey = "";

LORAE5 lorae5(devEUI, appEUI, appKey, devAddr, nwkSKey, appSKey);

const uint16_t DEVICE_ID = 1;
const uint16_t LOCATION_ID = 1;

// ========== SENSOR PROFILES ==========

// Predefined heater profiles and durations for BME68x sensors
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

// ========== SENSOR OBJECTS ==========

Bme68x bme[N_KIT_SENS];
comm_mux commSetup[N_KIT_SENS];
bool sensorActive[N_KIT_SENS] = {false};
uint32_t totalReadings[N_KIT_SENS] = {0};
uint32_t validReadings[N_KIT_SENS] = {0};
uint8_t currentStep = 0;
uint32_t profileCycles = 0;

// ========== GPS OBJECTS ==========

HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

// ========== SHARED GPS DATA AND MUTEX ==========

// Shared variables for latest GPS fix, protected by mutex
float shared_lat = 0.0, shared_lon = 0.0;
uint8_t shared_sats = 0;
uint32_t shared_gps_timestamp = 0;
SemaphoreHandle_t gpsMutex;

// ========== PROTOBUF ENCODING FOR SENSORS ==========

// Encodes and sends a BME68x sensor reading using Protobuf and LoRa
void publishSensorReadingLite(uint8_t sensor_idx, uint8_t measurement_step, float temp_c, float humidity_pct, float pressure_hpa, uint32_t gas_resistance_ohm, bool gas_valid, bool heat_stable, uint32_t timestamp_ms) {
  cedri_SensorReadingLite proto = cedri_SensorReadingLite_init_zero;
  proto.device_id = DEVICE_ID;
  proto.location_id = LOCATION_ID;
  proto.sensor_id = sensor_idx;
  proto.heater_profile = static_cast<cedri_HeaterProfile>(sensor_idx);
  proto.measurement_step = measurement_step;
  proto.temp_c = temp_c;
  proto.humidity_pct = humidity_pct;
  proto.pressure_hpa = pressure_hpa;
  proto.gas_resistance_ohm = gas_resistance_ohm;
  proto.gas_valid = gas_valid;
  proto.heat_stable = heat_stable;
  proto.timestamp = timestamp_ms;

  uint8_t buffer[64];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  if (pb_encode(&stream, cedri_SensorReadingLite_fields, &proto)) {
    Serial.printf("Encoded payload size: %zu bytes\n", stream.bytes_written);
    lorae5.sendData(buffer, stream.bytes_written);
    Serial.printf("✓ Sent %zu bytes over LoRa for sensor %d\n", stream.bytes_written, sensor_idx);
  } else {
    Serial.println("✗ Failed to encode Protobuf message");
  }
}

// ========== PROTOBUF ENCODING FOR GPS ==========

// Encodes and sends the latest GPS reading using Protobuf and LoRa
void publishGpsReading(float lat, float lon, uint8_t sats, uint32_t timestamp) {
  cedri_GpsReading proto = cedri_GpsReading_init_zero;
  proto.device_id = DEVICE_ID;
  proto.latitude = lat;
  proto.longitude = lon;
  proto.satellites = sats;
  proto.timestamp = timestamp;

  uint8_t buffer[32];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  if (pb_encode(&stream, cedri_GpsReading_fields, &proto)) {
    Serial.printf("[GPS] Encoded payload size: %zu bytes\n", stream.bytes_written);
    lorae5.sendData(buffer, stream.bytes_written);
    Serial.printf("[GPS] ✓ Sent %zu bytes over LoRa\n", stream.bytes_written);
  } else {
    Serial.println("[GPS] ✗ Failed to encode Protobuf message");
  }
}

// ========== GPS TASK ==========

// Continuously reads from the GPS serial port and updates the shared variables with the latest fix
void gpsTask(void*) {
  for (;;) {
    while (SerialGPS.available()) {
      gps.encode(SerialGPS.read());
    }
    if (gps.location.isValid()) {
      if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
        shared_lat = gps.location.lat();
        shared_lon = gps.location.lng();
        shared_sats = gps.satellites.value();
        shared_gps_timestamp = millis(); // Optionally, use gps.time.value() for UTC
        xSemaphoreGive(gpsMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
  }
}

// ========== LORA GPS TASK ==========

// Periodically sends the latest valid GPS fix over LoRa (if fix is good enough)
void loraGpsTask(void*) {
  for (;;) {
    float lat = 0.0, lon = 0.0;
    uint8_t sats = 0;
    uint32_t timestamp = 0;

    if (xSemaphoreTake(gpsMutex, portMAX_DELAY)) {
      lat = shared_lat;
      lon = shared_lon;
      sats = shared_sats;
      timestamp = shared_gps_timestamp;
      xSemaphoreGive(gpsMutex);
    }

    if (sats >= 3) { // Only send if GPS fix is reasonably reliable
      publishGpsReading(lat, lon, sats, timestamp);
    } else {
      Serial.println("[GPS] No valid fix available for LoRa transmission.");
    }

    vTaskDelay(pdMS_TO_TICKS(15000)); // Send every 15 seconds (adjust as needed)
  }
}

// ========== SENSOR ROUTINE TASKS ==========

// Triggers a heater cycle and waits for measurement
void triggerMeasurement(uint8_t id, uint8_t step) {
  if (!sensorActive[id]) return;
  uint16_t t = tempProfiles[id][step];
  uint16_t d = durProfiles[id][step];
  bme[id].setHeaterProf(t, d);
  bme[id].setOpMode(BME68X_FORCED_MODE);
  uint32_t us = bme[id].getMeasDur(BME68X_FORCED_MODE);
  delay((us + 999) / 1000);
}

// Collects all available data from sensors and sends it via LoRa
void collectResults(uint8_t step) {
  uint32_t now = millis();
  for (uint8_t i = 0; i < N_KIT_SENS; i++) {
    if (!sensorActive[i]) continue;

    Serial.printf("[Step %d] Fetching data from sensor %d...\n", step, i);

    if (bme[i].fetchData()) {
      bme68xData data;
      uint8_t left;
      do {
        left = bme[i].getData(data);
        if (data.status & BME68X_NEW_DATA_MSK) {
          totalReadings[i]++;
          bool gv = data.status & BME68X_GASM_VALID_MSK;
          bool hs = data.status & BME68X_HEAT_STAB_MSK;
          if (gv && hs) validReadings[i]++;
          publishSensorReadingLite(i, step, data.temperature, data.humidity, data.pressure, data.gas_resistance, gv, hs, now);
          delay(5000); // Delay between individual sensor transmissions
        }
      } while (left);
    }
  }
}

// Main sensor measurement state machine task
void measurementTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    collectResults(currentStep);
    currentStep = (currentStep + 1) % MAX_MEASUREMENTS;
    if (currentStep == 0) profileCycles++;

    for (uint8_t i = 0; i < N_KIT_SENS; i++) {
      if (sensorActive[i]) triggerMeasurement(i, currentStep);
    }

    delay(HEAT_STABILIZE);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEP_DURATION));
  }
}

// ========== INITIALIZATION ==========

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize the mutex for GPS data protection
  gpsMutex = xSemaphoreCreateMutex();

  // Initialize LoRa serial and LoRaWAN module
  Serial1.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(100);
  lorae5.setup_hardware(&Serial, &Serial1);
  lorae5.printInfo();

  lorae5.setup_lorawan(EU868, true, CLASS_A, 7, false, false, 8, false, 10000);
  Serial.print("[S] Joining LoRaWAN...");
  while (!lorae5.join()) {
    Serial.print(".");
    delay(2000);
  }
  Serial.println("\n✓ Joined LoRaWAN");

  // Initialize I2C, SPI, and multiplexed sensor communication
  Wire.begin(23, 22);
  Wire.setClock(400000);
  SPI.begin();
  comm_mux_begin(Wire, SPI);

  // Initialize all BME68x sensors
  for (uint8_t i = 0; i < N_KIT_SENS; i++) {
    commSetup[i] = comm_mux_set_config(Wire, SPI, i, commSetup[i]);
    bme[i].begin(BME68X_SPI_INTF, comm_mux_read, comm_mux_write, comm_mux_delay, &commSetup[i]);

    if (!bme[i].checkStatus()) {
      bme[i].setTPH();
      bme[i].setOpMode(BME68X_FORCED_MODE);
      sensorActive[i] = true;
      Serial.printf("✓ Sensor %d initialized successfully\n", i);
    } else {
      Serial.printf("✗ Sensor %d failed to initialize\n", i);
    }
  }

  // Initialize GPS module (HardwareSerial2)
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[GPS] GPS module initialized. Waiting for fix...");

  // Create the measurement, GPS, and LoRa GPS tasks
  const uint32_t MEASURE_TASK_STACK = 12288;
  xTaskCreatePinnedToCore(measurementTask, "Measure", MEASURE_TASK_STACK, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(gpsTask,         "GPSTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(loraGpsTask,     "LoRaGPS", 4096, NULL, 1, NULL, 1);
}

// ========== MAIN LOOP ==========

void loop() {
  // The main loop is left empty, as all work is handled by FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}