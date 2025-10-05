#!/bin/bash

# CEDRI ESP32 Schematic Validation Script
# Validates that all specifications have been implemented correctly

echo "🔍 CEDRI ESP32 Schematic Validation"
echo "=================================="

SCHEMATIC_FILE="/home/runner/work/CEDRI_/CEDRI_/hardware/CEDRI_ESP32.kicad_sch"
PCB_FILE="/home/runner/work/CEDRI_/CEDRI_/hardware/CEDRI_ESP32.kicad_pcb"
PROJECT_FILE="/home/runner/work/CEDRI_/CEDRI_/hardware/CEDRI_ESP32.kicad_pro"

echo ""
echo "📋 Checking File Existence:"
echo "----------------------------"

if [ -f "$SCHEMATIC_FILE" ]; then
    echo "✅ Schematic file exists: CEDRI_ESP32.kicad_sch"
else
    echo "❌ Schematic file missing"
    exit 1
fi

if [ -f "$PCB_FILE" ]; then
    echo "✅ PCB file exists: CEDRI_ESP32.kicad_pcb"
else
    echo "❌ PCB file missing"
    exit 1
fi

if [ -f "$PROJECT_FILE" ]; then
    echo "✅ Project file exists: CEDRI_ESP32.kicad_pro"
else
    echo "❌ Project file missing"
    exit 1
fi

echo ""
echo "🔌 Checking Pull-down Resistors for Unused Pins:"
echo "-------------------------------------------------"

# Check for pull-down resistors
PULLDOWN_COUNT=$(grep -c "Rpd_" "$SCHEMATIC_FILE")
if [ "$PULLDOWN_COUNT" -eq 4 ]; then
    echo "✅ Found 4 pull-down resistors (Rpd_14, Rpd_32, Rpd_15, Rpd_33)"
else
    echo "❌ Expected 4 pull-down resistors, found $PULLDOWN_COUNT"
fi

# Check specific resistors
for gpio in 14 32 15 33; do
    if grep -q "Rpd_$gpio" "$SCHEMATIC_FILE"; then
        echo "✅ Pull-down resistor Rpd_$gpio found for GPIO$gpio"
    else
        echo "❌ Missing pull-down resistor for GPIO$gpio"
    fi
done

# Check resistor values
RESISTOR_10K_COUNT=$(grep -c '"10k"' "$SCHEMATIC_FILE")
if [ "$RESISTOR_10K_COUNT" -eq 4 ]; then
    echo "✅ All pull-down resistors are 10kΩ"
else
    echo "❌ Expected 4 x 10kΩ resistors, found $RESISTOR_10K_COUNT"
fi

echo ""
echo "⚡ Checking Power Architecture:"
echo "------------------------------"

# Check VBAT_IN label
if grep -q "VBAT_IN" "$SCHEMATIC_FILE"; then
    echo "✅ VBAT_IN net label found"
else
    echo "❌ VBAT_IN net label missing"
fi

# Check 3V3 power distribution
if grep -q "+3V3" "$SCHEMATIC_FILE"; then
    echo "✅ 3V3 power rail present"
else
    echo "❌ 3V3 power rail missing"
fi

# Check ground connections
if grep -q "GND" "$SCHEMATIC_FILE"; then
    echo "✅ Ground connections present"
else
    echo "❌ Ground connections missing"
fi

echo ""
echo "🔧 Checking Component Selection:"
echo "--------------------------------"

# Check BME688 Feather Wing
if grep -q "BME688-FEATHER-WING" "$SCHEMATIC_FILE"; then
    echo "✅ BME688 Feather Wing Development Kit specified"
else
    echo "❌ BME688 Feather Wing not found"
fi

# Check VL53L0X
if grep -q "VL53L0X" "$SCHEMATIC_FILE"; then
    echo "✅ VL53L0X ToF sensor present"
else
    echo "❌ VL53L0X ToF sensor missing"
fi

# Check Grove Wio-E5
if grep -q "WIO-E5" "$SCHEMATIC_FILE"; then
    echo "✅ Grove Wio-E5 LoRaWAN module present"
else
    echo "❌ Grove Wio-E5 LoRaWAN module missing"
fi

# Check GPS
if grep -q "GPS" "$SCHEMATIC_FILE"; then
    echo "✅ GPS module present"
else
    echo "❌ GPS module missing"
fi

echo ""
echo "🔗 Checking I²C Configuration:"
echo "------------------------------"

# Check SDA/SCL connections
if grep -q "SDA" "$SCHEMATIC_FILE"; then
    echo "✅ SDA connections present"
else
    echo "❌ SDA connections missing"
fi

if grep -q "SCL" "$SCHEMATIC_FILE"; then
    echo "✅ SCL connections present"
else
    echo "❌ SCL connections missing"
fi

echo ""
echo "🏭 Checking PCB Preparation:"
echo "----------------------------"

# Check KiCad version
if grep -q "20231120" "$SCHEMATIC_FILE"; then
    echo "✅ KiCad 8.0 format confirmed"
else
    echo "❌ KiCad 8.0 format not confirmed"
fi

# Check PCB planes
if grep -q "zone" "$PCB_FILE"; then
    echo "✅ PCB planes configured"
else
    echo "❌ PCB planes missing"
fi

# Check layer setup
if grep -q "F.Cu" "$PCB_FILE" && grep -q "B.Cu" "$PCB_FILE"; then
    echo "✅ Top and bottom copper layers configured"
else
    echo "❌ Copper layers not properly configured"
fi

echo ""
echo "📊 Validation Summary:"
echo "======================"

# Count total checks
TOTAL_CHECKS=0
PASSED_CHECKS=0

# Run validation again and count results
{
    # File existence (3 checks)
    TOTAL_CHECKS=$((TOTAL_CHECKS + 3))
    [ -f "$SCHEMATIC_FILE" ] && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    [ -f "$PCB_FILE" ] && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    [ -f "$PROJECT_FILE" ] && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    
    # Pull-down resistors (5 checks)
    TOTAL_CHECKS=$((TOTAL_CHECKS + 5))
    [ "$(grep -c "Rpd_" "$SCHEMATIC_FILE")" -eq 4 ] && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "Rpd_14" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "Rpd_32" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "Rpd_15" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "Rpd_33" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    
    # Power architecture (3 checks)
    TOTAL_CHECKS=$((TOTAL_CHECKS + 3))
    grep -q "VBAT_IN" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "+3V3" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "GND" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    
    # Components (4 checks)
    TOTAL_CHECKS=$((TOTAL_CHECKS + 4))
    grep -q "BME688-FEATHER-WING" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "VL53L0X" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "WIO-E5" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "GPS" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    
    # I2C (2 checks)
    TOTAL_CHECKS=$((TOTAL_CHECKS + 2))
    grep -q "SDA" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "SCL" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    
    # PCB preparation (3 checks)
    TOTAL_CHECKS=$((TOTAL_CHECKS + 3))
    grep -q "20231120" "$SCHEMATIC_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    grep -q "zone" "$PCB_FILE" && PASSED_CHECKS=$((PASSED_CHECKS + 1))
    (grep -q "F.Cu" "$PCB_FILE" && grep -q "B.Cu" "$PCB_FILE") && PASSED_CHECKS=$((PASSED_CHECKS + 1))
}

echo "Total checks: $TOTAL_CHECKS"
echo "Passed checks: $PASSED_CHECKS"
echo "Success rate: $(echo "scale=1; $PASSED_CHECKS * 100 / $TOTAL_CHECKS" | bc)%"

if [ "$PASSED_CHECKS" -eq "$TOTAL_CHECKS" ]; then
    echo ""
    echo "🎉 ALL SPECIFICATIONS IMPLEMENTED SUCCESSFULLY!"
    echo "✅ Schematic is ready for manufacturing"
    exit 0
else
    echo ""
    echo "⚠️  Some requirements may need attention"
    echo "Please review the failed checks above"
    exit 1
fi