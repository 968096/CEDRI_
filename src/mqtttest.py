#!/usr/bin/env python3

import os
import signal
import logging
from datetime import datetime
import csv
import paho.mqtt.client as mqtt
from measurement_pb2 import SensorReading

BROKER    = "broker.emqx.io"
PORT      = 1883
TOPIC     = "home/sensors/bme688_sequential101"
OUT_FILE  = "bme688_data_protobuf.csv"
QOS       = 1

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("mqtt-csv-consumer")

# Prepare CSV header
CSV_FIELDS = [
    "device_id", "location", "volume_l", "sensor_id", "heater_profile",
    "measurement_step", "temp_c", "humidity_pct", "pressure_hpa",
    "gas_resistance_ohm", "gas_valid", "heat_stable", "timestamp"
]
if not os.path.isfile(OUT_FILE):
    with open(OUT_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_FIELDS)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("Connected, subscribing to %s", TOPIC)
        client.subscribe(TOPIC, qos=QOS)
    else:
        logger.error("Connect failed (rc=%d)", rc)

def on_message(client, userdata, msg):
    try:
        reading = SensorReading()
        reading.ParseFromString(msg.payload)
        line = [
            reading.device_id,
            reading.location,
            reading.volume_l,
            reading.sensor_id,
            reading.heater_profile,
            reading.measurement_step,
            reading.temp_c,
            reading.humidity_pct,
            reading.pressure_hpa,
            reading.gas_resistance_ohm,
            int(reading.gas_valid),
            int(reading.heat_stable),
            reading.timestamp
        ]
        with open(OUT_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(line)
        print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] → Wrote row: {line}")
    except Exception as e:
        logger.error("Failed to parse Protobuf message: %s", e)

def on_disconnect(client, userdata, rc):
    if rc != 0:
        logger.warning("Unexpected disconnect (rc=%d)", rc)
    else:
        logger.info("Clean disconnect")

client = mqtt.Client(client_id=f"ProtobufConsumer_{os.getpid()}")
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

def _shutdown(signum, frame):
    logger.info("Shutting down…")
    client.disconnect()
    exit(0)

signal.signal(signal.SIGINT,  _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

logger.info("Connecting to %s:%d…", BROKER, PORT)
client.connect(BROKER, PORT, keepalive=60)
client.loop_forever()