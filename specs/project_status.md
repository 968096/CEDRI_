# Project Status: ESP32 BME688 + ToF + LoRaWAN

## Overview

This project implements a multi-sensor data acquisition system using:
- 8 BME688 sensors (each with a unique heater profile)
- 1 VL53L0X ToF sensor for reservoir volume
- LoRaWAN uplink (Seeed Wio-E5 module)
- Optional GPS (TinyGPSPlus)
- RTOS (FreeRTOS) for concurrency

## Sensor Profiles

- **Current:** 8 heater profiles, each mapped to a BME688 sensor (see `tempProfiles` and `durProfiles` in `main.cpp`)
- **Note:** There are 8 additional profiles available for future expansion.

## RTOS Synchronization and Concurrency Improvements

Recent changes (as of 2024-06):

### Issues Addressed
- [x] SPI/I2C bus access not protected (shared by all sensors)
- [x] LoRaWAN transmission not synchronized
- [x] Replaced `delay()` with `vTaskDelay()` in tasks
- [x] Improved task core distribution (GPS task moved to core 0)
- [x] Serial output mutex to prevent interleaved prints
- [x] Timeout/error handling for mutex operations

### Synchronization Primitives Added
- `spiBusMutex`: Protects SPI bus for sensor comms
- `i2cBusMutex`: Protects I2C bus for chip select
- `lorawanMutex`: Serializes LoRaWAN transmissions
- `serialMutex`: Serial output protection

### Implementation Notes
- All shared resource access is now protected by mutexes/semaphores.
- Error handling for mutex acquisition (timeouts, failures).
- Minimal changes to preserve existing logic.

---

**Keep this file updated as the project evolves.**
