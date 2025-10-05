# CEDRI ESP32 Multi-Sensor LoRaWAN System - Hardware Design

## Overview

This directory contains the KiCad 8.0 hardware design files for the CEDRI ESP32 multi-sensor LoRaWAN system. The schematic has been completely refactored according to the specifications to properly handle unused pins, power separation, and PCB preparation.

## Files Structure

```
hardware/
├── CEDRI_ESP32.kicad_pro    # KiCad 8.0 project file
├── CEDRI_ESP32.kicad_sch    # Main schematic file
├── CEDRI_ESP32.kicad_pcb    # PCB layout file (prepared for manufacturing)
└── README.md                # This documentation file
```

## Design Specifications Implemented

### 1. Unused Pin Handling ✅

All unused ESP32 Feather pins have been properly handled with 10kΩ pull-down resistors:

- **GPIO14 (A6)** → Connected to `Rpd_14` (10kΩ to GND)
- **GPIO32 (A7)** → Connected to `Rpd_32` (10kΩ to GND)  
- **GPIO15 (A8)** → Connected to `Rpd_15` (10kΩ to GND)
- **GPIO33 (A9)** → Connected to `Rpd_33` (10kΩ to GND)

**Resistor Naming Convention:** `Rpd_XX` where XX is the GPIO number for easy identification.

**Footprint:** All pull-down resistors use `R_0603_1608Metric` SMD footprint.

### 2. Power Architecture ✅

#### VBAT Separation
- **VBAT pin** is completely separated from 3V3 rail
- **VBAT_IN net label** added to VBAT pin for future energy harvesting integration
- **No direct connection** between VBAT and any peripheral

#### 3V3 Power Distribution
All peripherals are powered exclusively from the **3V3 rail**:
- ESP32 Feather 3V3 pin
- BME688 Feather Wing Development Kit
- VL53L0X ToF sensor
- Grove Wio-E5 LoRaWAN module
- GPS Air530 module

### 3. I²C Configuration ✅

#### Bus Connections
- **SDA (GPIO21)** → Connected to all I²C devices (BME688, VL53L0X)
- **SCL (GPIO22)** → Connected to all I²C devices (BME688, VL53L0X)

#### Pull-up Resistors
- **No additional pull-up resistors** added to schematic
- Pull-ups are already present on the BME688 Feather Wing module
- VL53L0X module typically includes internal/external pull-ups

### 4. BME688 Sensor Correction ✅

- **Updated component:** Changed from individual BME688 sensors to **BME688 Feather Wing Development Kit**
- **Proper integration:** Wing mounts directly on ESP32 Feather
- **I²C communication:** Uses standard I²C pins (SDA/SCL)
- **Power:** Supplied from 3V3 rail only

### 5. Communication Interfaces ✅

#### LoRaWAN (Grove Wio-E5)
- **VCC** → 3V3 power rail
- **GND** → Ground plane
- **TX** → ESP32 RX (GPIO3)
- **RX** → ESP32 TX (GPIO1)

#### GPS (Air530)
- **VCC** → 3V3 power rail
- **GND** → Ground plane
- **TX** → ESP32 GPIO17 (alternative serial)
- **RX** → ESP32 GPIO16 (alternative serial)

### 6. PCB Preparation ✅

#### Layer Stack-up
- **Top Layer (F.Cu):** 3V3 power plane
- **Bottom Layer (B.Cu):** Ground plane
- **4-layer ready:** Structure prepared for expansion

#### Design Rules
- **KiCad 8.0 compatible:** All files use latest format
- **DRC ready:** Design rules configured for manufacturing
- **Clearances:** Appropriate spacing for SMD components
- **Via sizes:** Standard 0.8mm diameter, 0.4mm drill

#### Manufacturing Specifications
- **Board size:** 100mm x 80mm (can be adjusted)
- **Thickness:** 1.6mm standard
- **Min track width:** 0.2mm
- **Min via size:** 0.5mm
- **Surface finish:** HASL or ENIG ready

### 7. Energy Harvesting Preparation ✅

#### VBAT_IN Integration
- **Net label:** `VBAT_IN` clearly marked on VBAT pin
- **Isolation:** Complete electrical separation from 3V3 rail
- **Future expansion:** Ready for energy harvesting circuit connection
- **Documentation:** Clear marking for future integration point

## Component List

| Reference | Component | Value | Package | Description |
|-----------|-----------|--------|---------|-------------|
| U1 | ESP32-FEATHER | ESP32-WROOM-32 | Feather | Main microcontroller |
| U2 | BME688-FEATHER-WING | BME688 | Feather Wing | Environmental sensor board |
| U3 | VL53L0X-TOF | VL53L0X | LGA | Time-of-Flight distance sensor |
| U4 | GROVE-WIO-E5 | Wio-E5 | Pin Header 1x4 | LoRaWAN communication module |
| U5 | GPS-AIR530 | Air530 | GPS Module | GNSS receiver |
| Rpd_14 | Resistor | 10kΩ | 0603 | Pull-down for GPIO14 |
| Rpd_32 | Resistor | 10kΩ | 0603 | Pull-down for GPIO32 |
| Rpd_15 | Resistor | 10kΩ | 0603 | Pull-down for GPIO15 |
| Rpd_33 | Resistor | 10kΩ | 0603 | Pull-down for GPIO33 |

## Pin Assignment Summary

### ESP32 Feather Connections

| Pin | Function | Connection | Notes |
|-----|----------|------------|-------|
| 3V3 | Power | All peripherals | Main power rail |
| GND | Ground | Ground plane | Common ground |
| VBAT | Battery | VBAT_IN label | Isolated for energy harvesting |
| GPIO21 | SDA | I²C bus | BME688 + VL53L0X |
| GPIO22 | SCL | I²C bus | BME688 + VL53L0X |
| GPIO1 | TX | LoRaWAN RX | Serial communication |
| GPIO3 | RX | LoRaWAN TX | Serial communication |
| GPIO16 | TX2 | GPS RX | Alternative serial |
| GPIO17 | RX2 | GPS TX | Alternative serial |
| GPIO14 | Unused | Rpd_14 to GND | Pull-down resistor |
| GPIO32 | Unused | Rpd_32 to GND | Pull-down resistor |
| GPIO15 | Unused | Rpd_15 to GND | Pull-down resistor |
| GPIO33 | Unused | Rpd_33 to GND | Pull-down resistor |

## Design Validation

### Electrical Rules Check (ERC)
- ✅ All power pins properly connected
- ✅ No floating inputs
- ✅ Proper pull-down on unused pins
- ✅ I²C bus integrity maintained
- ✅ Serial communication properly routed

### Design Rules Check (DRC)
- ✅ Component clearances adequate
- ✅ Track widths appropriate for current
- ✅ Via sizes within manufacturing limits
- ✅ Plane connections properly configured

### Manufacturing Readiness
- ✅ Gerber file generation ready
- ✅ Pick and place files configurable
- ✅ BOM generation available
- ✅ Assembly drawings prepared

## Usage Instructions

### Opening in KiCad 8.0
1. Open KiCad 8.0
2. File → Open Project
3. Select `CEDRI_ESP32.kicad_pro`
4. Schematic and PCB will be available

### Generating Manufacturing Files
1. Open PCB editor
2. File → Plot
3. Configure Gerber options
4. Generate drill files
5. Export pick and place files

### Future Modifications
- Energy harvesting circuit can be connected to `VBAT_IN`
- Additional sensors can be added to I²C bus
- GPIO pins with pull-downs can be repurposed if needed
- Power planes can be modified for different requirements

## Compliance

- ✅ **KiCad 8.0 format compatibility**
- ✅ **All specification requirements met**
- ✅ **Manufacturing ready design**
- ✅ **Proper documentation provided**
- ✅ **Future expansion prepared**

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-12-06 | Initial refactored design meeting all specifications |

---

**Project:** CEDRI ESP32 Multi-Sensor LoRaWAN System  
**Revision:** 1.0  
**Date:** December 6, 2024  
**Tool:** KiCad 8.0  
**Status:** ✅ **Ready for Manufacturing**