#!/bin/bash
# Basic verification script for CEDRI ESP32 FreeRTOS improvements

echo "=== CEDRI ESP32 FreeRTOS Synchronization Verification ==="
echo

# Check for required includes
echo "1. Checking required includes..."
if grep -q "#include <cstdarg>" src/main.cpp; then
    echo "   ✓ cstdarg included for va_list"
else
    echo "   ✗ cstdarg missing"
fi

if grep -q "#include \"freertos/semphr.h\"" src/main.cpp; then
    echo "   ✓ FreeRTOS semaphore header included"
else
    echo "   ✗ FreeRTOS semaphore header missing"
fi

# Check mutex declarations
echo
echo "2. Checking mutex declarations..."
mutexes=("gpsMutex" "spiMutex" "i2cMutex" "lorawanMutex" "serialMutex" "sensorStateMutex")
for mutex in "${mutexes[@]}"; do
    if grep -q "SemaphoreHandle_t $mutex;" src/main.cpp; then
        echo "   ✓ $mutex declared"
    else
        echo "   ✗ $mutex missing"
    fi
done

# Check mutex creation
echo
echo "3. Checking mutex creation in setup()..."
for mutex in "${mutexes[@]}"; do
    if grep -q "$mutex = xSemaphoreCreateMutex();" src/main.cpp; then
        echo "   ✓ $mutex created"
    else
        echo "   ✗ $mutex creation missing"
    fi
done

# Check timeout definition
echo
echo "4. Checking timeout configuration..."
if grep -q "MUTEX_TIMEOUT_MS.*1000" src/main.cpp; then
    echo "   ✓ Mutex timeout configured (1000ms)"
else
    echo "   ✗ Mutex timeout not configured"
fi

# Check thread-safe helper functions
echo
echo "5. Checking thread-safe helper functions..."
helpers=("getSensorState" "setSensorState" "safePrintln" "safe_comm_mux_read" "safe_comm_mux_write")
for helper in "${helpers[@]}"; do
    if grep -q "^[a-zA-Z_][a-zA-Z0-9_]* $helper(" src/main.cpp; then
        echo "   ✓ $helper function implemented"
    else
        echo "   ✗ $helper function missing"
    fi
done

# Check task core assignments
echo
echo "6. Checking task core assignments..."
if grep -q "xTaskCreatePinnedToCore(measurementTask.*NULL, 1)" src/main.cpp; then
    echo "   ✓ Measurement task on core 1"
else
    echo "   ✗ Measurement task core assignment incorrect"
fi

if grep -q "xTaskCreatePinnedToCore(gpsTask.*NULL, 0)" src/main.cpp; then
    echo "   ✓ GPS task on core 0"
else
    echo "   ✗ GPS task core assignment incorrect"
fi

# Check portMAX_DELAY removal
echo
echo "7. Checking portMAX_DELAY removal..."
if grep -q "portMAX_DELAY" src/main.cpp; then
    echo "   ✗ portMAX_DELAY still present (should be replaced with timeouts)"
else
    echo "   ✓ portMAX_DELAY removed"
fi

# Check error handling
echo
echo "8. Checking error handling..."
if grep -q "mutex timeout" src/main.cpp; then
    echo "   ✓ Mutex timeout error handling present"
else
    echo "   ✗ Mutex timeout error handling missing"
fi

echo
echo "=== Verification Complete ==="
echo "If all checks pass, the FreeRTOS synchronization improvements are correctly implemented."