#ifndef PROTOBUF_HANDLER_H
#define PROTOBUF_HANDLER_H

#include <pb_encode.h>
#include <pb_decode.h>
#include "measurement.pb.h"
#include <string.h>

// Externs for your global device info
extern const char* DEVICE_ID;
extern const char* LOCATION;
extern const float VOLUME_L;

// Callback for encoding strings
static bool encode_string(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    const char *str = (const char*)(*arg);
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    return pb_encode_string(stream, (const uint8_t*)str, strlen(str));
}

class ProtobufHandler {
public:
    static bool packSensorReading(
        uint8_t id, const char* profile,
        uint8_t step, float temp_c,
        float humidity_pct, float pressure_hpa,
        uint32_t gas_res_ohm, bool gas_valid,
        bool heat_stable, uint64_t timestamp,
        uint8_t* buffer, size_t buffer_size, size_t* message_length
    ) {
        cedri_SensorReading message = cedri_SensorReading_init_zero;

        // Set up callback for each string
        message.device_id.funcs.encode = &encode_string;
        message.device_id.arg = (void*)DEVICE_ID;

        message.location.funcs.encode = &encode_string;
        message.location.arg = (void*)LOCATION;

        message.heater_profile.funcs.encode = &encode_string;
        message.heater_profile.arg = (void*)profile;

        message.volume_l = VOLUME_L;
        message.sensor_id = id;
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