#!/usr/bin/env python3
"""
Verification script for the CEDRI ESP32 clean schematic design.
Validates that all required connections are present and properly labeled.
"""

import re
import sys
from pathlib import Path

def verify_schematic():
    """Verify the schematic contains all required connections and labels."""
    
    schematic_file = Path(__file__).parent / "raicycle.kicad_sch"
    
    if not schematic_file.exists():
        print("❌ ERROR: raicycle.kicad_sch not found!")
        return False
    
    with open(schematic_file, 'r') as f:
        content = f.read()
    
    print("🔍 Verifying CEDRI ESP32 Clean Schematic Design...")
    print("=" * 60)
    
    # Check for required global labels
    required_labels = [
        "3V3", "GND", 
        "GPS_TX", "GPS_RX",
        "LORA_TX", "LORA_RX", 
        "I2C_SCL", "I2C_SDA"
    ]
    
    print("\n📋 Checking Global Labels:")
    all_labels_found = True
    for label in required_labels:
        pattern = rf'global_label "{label}"'
        if re.search(pattern, content):
            print(f"✅ {label:<10} - Found")
        else:
            print(f"❌ {label:<10} - Missing")
            all_labels_found = False
    
    # Check for required components
    required_components = [
        ("ESP32", "ESP32-S3-DevKitC-1", "U1"),
        ("BME688", "BME688", "U2"), 
        ("VL53L0X", "VL53L0X", "U3"),
        ("GPS", "GPS_Air530", "U4"),
        ("LoRaWAN", "LoRaWAN_WioE5", "U5")
    ]
    
    print("\n🔧 Checking Components:")
    all_components_found = True
    for comp_name, lib_name, ref in required_components:
        if lib_name in content and ref in content:
            print(f"✅ {comp_name:<12} ({ref}) - Found")
        else:
            print(f"❌ {comp_name:<12} ({ref}) - Missing")
            all_components_found = False
    
    # Check for proper KiCad format
    print("\n📄 Checking File Format:")
    format_checks = [
        ("KiCad Header", r"\(kicad_sch \(version"),
        ("UUID", r"\(uuid [0-9a-f-]+\)"),
        ("Paper Size", r'\(paper "A4"\)'),
        ("Library Symbols", r"\(lib_symbols"),
        ("Wire Connections", r"\(wire \(pts"),
        ("Junctions", r"\(junction \(at")
    ]
    
    all_format_ok = True
    for check_name, pattern in format_checks:
        if re.search(pattern, content):
            print(f"✅ {check_name:<15} - OK")
        else:
            print(f"❌ {check_name:<15} - Missing")
            all_format_ok = False
    
    # Check pin mappings match code requirements
    print("\n🔌 Verifying Pin Mappings:")
    pin_mappings = [
        ("GPIO16", "LORA_RX"),
        ("GPIO17", "LORA_TX"), 
        ("GPIO22", "I2C_SCL"),
        ("GPIO23", "I2C_SDA"),
        ("GPIO32", "GPS_TX"),
        ("GPIO33", "GPS_RX")
    ]
    
    all_pins_ok = True
    for gpio, signal in pin_mappings:
        if gpio in content and signal in content:
            print(f"✅ {gpio:<8} → {signal:<10} - Mapped")
        else:
            print(f"❌ {gpio:<8} → {signal:<10} - Missing")
            all_pins_ok = False
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 VERIFICATION SUMMARY:")
    print("=" * 60)
    
    checks = [
        ("Global Labels", all_labels_found),
        ("Components", all_components_found), 
        ("File Format", all_format_ok),
        ("Pin Mappings", all_pins_ok)
    ]
    
    all_passed = True
    for check_name, passed in checks:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{check_name:<15}: {status}")
        if not passed:
            all_passed = False
    
    print("\n" + "=" * 60)
    if all_passed:
        print("🎉 OVERALL RESULT: ✅ ALL CHECKS PASSED!")
        print("   The schematic is clean, properly labeled, and ready for use.")
        return True
    else:
        print("⚠️  OVERALL RESULT: ❌ SOME CHECKS FAILED!")
        print("   Please review and fix the issues above.")
        return False

def main():
    """Main function."""
    success = verify_schematic()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()