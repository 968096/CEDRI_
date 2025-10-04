# CEDRI ESP32 Multi-Sensor System - Clean Schematic Design

## Overview
This document describes the clean, professional schematic design implemented for the CEDRI ESP32 multi-sensor IoT system. The schematic eliminates crossing wires by using proper labels and implements best practices for readability and maintainability.

## Schematic Architecture

### Components Layout
The schematic is organized with components positioned logically:

1. **ESP32 Feather (U1)** - Positioned centrally as the main controller
2. **BME688 Development Kit (U2)** - Environmental sensor array (left side)
3. **VL53L0X ToF Sensor (U3)** - Volume measurement sensor (upper right)
4. **GPS Air530 (U4)** - GPS module (lower right)
5. **Grove Wio-E5 LoRaWAN (U5)** - Communication module (lower left)

### Clean Design Principles Applied

#### 1. Label-Based Connections
Instead of crossing wires, all connections use proper labels:

**Power Rails:**
- `3V3` - 3.3V power supply to all modules
- `GND` - Ground connections for all modules

**Communication Lines:**
- `GPS_TX` / `GPS_RX` - GPS UART communication
- `LORA_TX` / `LORA_RX` - LoRaWAN UART communication  
- `I2C_SCL` / `I2C_SDA` - I2C bus for BME688 and VL53L0X sensors

#### 2. Pin Mapping (from code analysis)

**ESP32 Feather Connections:**
```
Pin 16 (GPIO16) → LORA_RX    (LoRaWAN module RX)
Pin 17 (GPIO17) → LORA_TX    (LoRaWAN module TX)
Pin 22 (GPIO22) → I2C_SCL    (I2C Clock for sensors)
Pin 23 (GPIO23) → I2C_SDA    (I2C Data for sensors)
Pin 32 (GPIO32) → GPS_TX     (GPS module RX input)
Pin 33 (GPIO33) → GPS_RX     (GPS module TX output)
3V3 Pin        → 3V3         (Power to all modules)
GND Pin        → GND         (Ground for all modules)
```

**Module Connections:**
- **BME688 Dev Kit**: VCC→3V3, GND→GND, SCL→I2C_SCL, SDA→I2C_SDA
- **VL53L0X ToF**: VCC→3V3, GND→GND, SCL→I2C_SCL, SDA→I2C_SDA  
- **GPS Air530**: VCC→3V3, GND→GND, RX→GPS_TX, TX→GPS_RX
- **Grove Wio-E5**: VCC→3V3, GND→GND, RX→LORA_TX, TX→LORA_RX

### 3. Professional Features

#### Power Distribution
- Clean power rails using junctions and labels
- Single 3V3 power source distributed to all modules
- Common ground plane for all components

#### Signal Organization
- Communication signals grouped by protocol (UART, I2C)
- Descriptive net names for easy PCB routing
- No crossing wires - all connections via labels

#### Component Placement
- Logical grouping: power management, communication, sensors
- Clear visual hierarchy with main controller central
- Adequate spacing for readability

## Net Names for PCB Routing

The schematic provides proper net names that will facilitate clean PCB routing:

### Power Nets
- `3V3` - 3.3V power distribution
- `GND` - Ground plane

### Communication Nets  
- `GPS_TX` - ESP32 Pin 32 to GPS RX
- `GPS_RX` - ESP32 Pin 33 to GPS TX
- `LORA_TX` - ESP32 Pin 17 to LoRaWAN RX
- `LORA_RX` - ESP32 Pin 16 to LoRaWAN TX
- `I2C_SCL` - ESP32 Pin 22 to all I2C devices SCL
- `I2C_SDA` - ESP32 Pin 23 to all I2C devices SDA

## Schematic Benefits

### Readability
- ✅ No crossing wires
- ✅ Clear component labels and values
- ✅ Logical component placement
- ✅ Consistent net naming

### Maintainability  
- ✅ Easy to trace connections
- ✅ Simple to modify or add components
- ✅ Clear documentation of pin assignments
- ✅ Professional appearance

### PCB Design Ready
- ✅ Proper net names for routing
- ✅ Clear power distribution
- ✅ Organized signal groups
- ✅ No ambiguous connections

## Hardware Compatibility

This schematic matches the existing firmware implementation:
- Pin assignments match the `main.cpp` definitions
- I2C bus shared between BME688 and VL53L0X (as implemented with mutex protection)
- UART assignments for GPS and LoRaWAN modules
- Power distribution compatible with ESP32 Feather 3.3V supply

## Files Created

1. `raicycle.kicad_sch` - Main schematic file with clean design
2. `raicycle.kicad_pro` - KiCad project file with proper settings
3. `SCHEMATIC_README.md` - This documentation file

The schematic can be opened in KiCad 7.0+ and is ready for PCB layout design with clean, professional routing.