#!/usr/bin/env python3
"""
MQTT → CSV logger for ESP32‐BME688 sequential measurements
with column headers and reliable appends.
"""

import os
import signal
import logging
from datetime import datetime
import paho.mqtt.client as mqtt
import google.protobuf.json_format as json_format
from measurement_pb2 import SensorReading

# ──────────────────────────────────────────────────────────────
# 🔧  USER‐EDITABLE SETTINGS
# ──────────────────────────────────────────────────────────────
BROKER    = "broker.emqx.io"
PORT      = 1883
TOPIC     = "home/sensors/bme688_sequential101"
OUT_FILE  = "bme688_data_protobuf.csv"
QOS       = 1               # 0 = at most once, 1 = at least once

# ──────────────────────────────────────────────────────────────
# 📋  LOGGER SETUP
# ──────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("mqtt-csv-consumer")

# ──────────────────────────────────────────────────────────────
# 🪝  CALLBACKS
# ──────────────────────────────────────────────────────────────
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
        line = json_format.MessageToJson(reading)
        with open(OUT_FILE, "a", encoding="utf-8") as f:
            f.write(line + "\n")
        print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] → Wrote: {line}")
    except Exception as e:
        logger.error("Failed to parse Protobuf message: %s", e)

def on_disconnect(client, userdata, rc):
    if rc != 0:
        logger.warning("Unexpected disconnect (rc=%d)", rc)
    else:
        logger.info("Clean disconnect")

# ──────────────────────────────────────────────────────────────
# 🚀  MAIN
# ──────────────────────────────────────────────────────────────
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