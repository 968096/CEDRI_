# CEDRI ESP32 System - Clean Schematic Layout

## Visual Layout Overview

```
                                   3V3 Power Rail
                    ┌──────────────────────────────────────────────┐
                    │                                              │
                    │              ┌─────────────────┐             │
                    └─────────────►│   VL53L0X ToF   │             │
                                   │     (U3)        │             │
                                   │  SCL      SDA   │             │
                                   └──┬────────────┬─┘             │
                                      │            │               │
                                      │            │               │
                    I2C_SCL ◄─────────┘            └──────► I2C_SDA│
                         ▲                                        ▲ │
                         │                                        │ │
┌────────────────────────┼────────────────────────────────────────┼─┘
│                        │                                        │
│   ┌─────────────────┐  │         ┌─────────────────┐            │
│   │   BME688 Kit    │  │         │   ESP32 Feather │            │
│   │     (U2)        │  │         │      (U1)       │            │
│   │                 │  │         │                 │            │
│   │  SCL      SDA   │  │         │ 16 17 22 23 32 33│            │
│   └──┬────────────┬─┘  │         │ │  │  │  │  │  │ │            │
│      │            │    │         │ │  │  │  │  │  │ │            │
│      └────────────┼────┼─────────┘ │  │  └──┼──┼──┘ │            │
│                   │    └───────────┘  │     │  │    │            │
│                   │                   │     │  │    │            │
│                   └───────────────────┼─────┘  │    │            │
│                                       │        │    │            │
│                                       │        │    │            │
│   ┌─────────────────┐                 │        │    │            │
└──►│  Grove Wio-E5   │                 │        │    │            │
    │  LoRaWAN (U5)   │                 │        │    │            │
    │                 │                 │        │    │            │
    │   RX      TX    │                 │        │    │            │
    └────┬────────┬───┘                 │        │    │            │
         │        │                     │        │    │            │
         │        │                     │        │    │            │
LORA_RX ◄┘        └─────────────────────┘        │    │            │
                                                 │    │            │
                                                 │    │            │
                  ┌─────────────────┐            │    │            │
                  │   GPS Air530    │            │    │            │
                  │     (U4)        │            │    │            │
                  │                 │            │    │            │
                  │   RX      TX    │            │    │            │
                  └────┬────────┬───┘            │    │            │
                       │        │                │    │            │
                       │        │                │    │            │
              GPS_TX ◄─┘        └────────────────┘    │            │
                                                      │            │
                                                      │            │
                                             GPS_RX ◄─┘            │
                                                                   │ 
                                                                   │
                              GND Rail ◄───────────────────────────┘
```

## Signal Flow Diagram

```
ESP32 Feather (U1) - Main Controller
├── Power Distribution
│   ├── 3V3 → [All Modules VCC]
│   └── GND → [All Modules GND]
│
├── UART Communications
│   ├── GPIO16 (LORA_RX) ← Grove Wio-E5 TX
│   ├── GPIO17 (LORA_TX) → Grove Wio-E5 RX
│   ├── GPIO32 (GPS_TX)  → GPS Air530 RX
│   └── GPIO33 (GPS_RX)  ← GPS Air530 TX
│
└── I2C Bus (shared)
    ├── GPIO22 (I2C_SCL) → [BME688 Kit SCL, VL53L0X SCL]
    └── GPIO23 (I2C_SDA) → [BME688 Kit SDA, VL53L0X SDA]
```

## Clean Design Benefits

### ❌ Before (Traditional Messy Design)
```
     ┌─────┐    ╱╲    ┌─────┐
     │ESP32├────╱──╲───┤ GPS │
     │     │   ╱    ╲  │     │
     │     ├──╱──────╲─┤LoRa │
     │     │ ╱    ╱╲  ╲│     │
     │     ├╱────╱──╲──┤BME  │
     └─────┘    ╱    ╲ └─────┘
              ╱   ╱╲  ╲
             ╱   ╱  ╲  ╲
            ╱   ╱    ╲  ╲
           ╱   ╱      ╲  ╲
```
*Crossing wires everywhere - difficult to trace, error-prone*

### ✅ After (Clean Label-Based Design)
```
3V3 ═══════════════════════════════════
     │         │         │         │
┌─────┴──┐ ┌────┴───┐ ┌────┴───┐ ┌────┴───┐
│ ESP32  │ │  GPS   │ │ LoRa   │ │ BME688 │
│        │ │        │ │        │ │        │
│ GPS_TX ├─┤ RX     │ │        │ │        │
│ GPS_RX ├─┤ TX     │ │        │ │        │
│LORA_TX ├─┼────────┼─┤ RX     │ │        │
│LORA_RX ├─┼────────┼─┤ TX     │ │        │
│I2C_SCL ├─┼────────┼─┼────────┼─┤ SCL    │
│I2C_SDA ├─┼────────┼─┼────────┼─┤ SDA    │
└─────┬──┘ └────┬───┘ └────┬───┘ └────┬───┘
      │         │          │          │
GND ══════════════════════════════════════
```
*Clean labels - easy to trace, professional appearance*

## Net Names Summary

| Net Name  | ESP32 Pin | Connected To          | Function              |
|-----------|-----------|----------------------|----------------------|
| 3V3       | 3V3       | All module VCC pins  | Power supply         |
| GND       | GND       | All module GND pins  | Ground reference     |
| GPS_TX    | GPIO32    | GPS Air530 RX       | GPS command/config   |
| GPS_RX    | GPIO33    | GPS Air530 TX       | GPS data reception   |
| LORA_TX   | GPIO17    | Grove Wio-E5 RX      | LoRaWAN commands     |
| LORA_RX   | GPIO16    | Grove Wio-E5 TX      | LoRaWAN responses    |
| I2C_SCL   | GPIO22    | BME688 & VL53L0X SCL | I2C clock line       |
| I2C_SDA   | GPIO23    | BME688 & VL53L0X SDA | I2C data line        |

## Component References

| Reference | Component       | Function                          |
|-----------|-----------------|-----------------------------------|
| U1        | ESP32 Feather   | Main microcontroller             |
| U2        | BME688 Dev Kit  | Environmental sensor array       |
| U3        | VL53L0X ToF     | Time-of-flight distance sensor   |
| U4        | GPS Air530      | GPS location receiver            |
| U5        | Grove Wio-E5    | LoRaWAN communication module     |

This clean design eliminates all crossing wires while preserving all functionality, making the schematic professional, readable, and ready for PCB layout.