#ifndef PROTOBUF_HANDLER_H
#define PROTOBUF_HANDLER_H

#include <pb_encode.h>
#include <pb_decode.h>
#include  <measurement.pb.h>
#include <string.h>

class ProtobufHandler {
public:
    static bool packSensorReading(
        uint8_t id, const char* profile,
        uint8_t step, float temp_c,
        float humidity_pct, float pressure_hpa,
        uint32_t gas_res_ohm, bool gas_valid,
        bool heat_stable, uint32_t timestamp,
        uint8_t* buffer, size_t buffer_size, size_t* message_length
    ) {
        cedri_SensorReading message = cedri_SensorReading_init_zero;

        strncpy(message.device_id, DEVICE_ID, sizeof(message.device_id));
        strncpy(message.location, LOCATION, sizeof(message.location));
        message.volume_l = VOLUME_L;
        message.sensor_id = id;
        strncpy(message.heater_profile, profile, sizeof(message.heater_profile));
        message.measurement_step = step;
        message.temp_c = temp_c;
        message.humidity_pct = humidity_pct;
        message.pressure_hpa = pressure_hpa;
        message.gas_resistance_ohm = gas_res_ohm;
        message.gas_valid = gas_valid;
        message.heat_stable = heat_stable;
        message.timestamp = timestamp;

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
        bool status = pb_encode(&stream, cedri_SensorReading_fields, &message);
        *message_length = stream.bytes_written;

        return status;
    }
};

#endif