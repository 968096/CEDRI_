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

# ──────────────────────────────────────────────────────────────
# 🔧  USER‐EDITABLE SETTINGS
# ──────────────────────────────────────────────────────────────
BROKER    = "broker.emqx.io"
PORT      = 1883
TOPIC     = "home/sensors/bme688_sequential11"
OUT_FILE  = "bme688_data_second_8_profiles_AIR.csv"
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
# 🏷  CSV HEADER (must match producer format)
# ──────────────────────────────────────────────────────────────
CSV_HEADER = ",".join([
    "device_id",
    "location",
    "volume_l",
    "sensor_id",
    "heater_profile",
    "measurement_step",
    "temp_c",
    "humidity_pct",
    "pressure_hpa",
    "gas_resistance_ohm",
    "gas_valid",
    "heat_stable",
    "timestamp"
])

# ──────────────────────────────────────────────────────────────
# 🪝  CALLBACKS
# ──────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("Connected, subscribing to %s", TOPIC)
        client.subscribe(TOPIC, qos=QOS)
    else:
        logger.error("Connect failed (rc=%d)", rc)

def on_subscribe(client, userdata, mid, granted_qos):
    logger.info("Subscribed (mid=%d, qos=%s)", mid, granted_qos)

def on_message(client, userdata, msg):
    line = msg.payload.decode("utf-8").strip()
    # if file doesn't exist or is empty, write header first
    write_header = not os.path.exists(OUT_FILE) or os.path.getsize(OUT_FILE) == 0

    with open(OUT_FILE, "a", encoding="utf-8") as f:
        if write_header:
            f.write(CSV_HEADER + "\n")
        f.write(line + "\n")

    print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] → Wrote: {line}")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        logger.warning("Unexpected disconnect (rc=%d)", rc)
    else:
        logger.info("Clean disconnect")

# ──────────────────────────────────────────────────────────────
# 🚀  MAIN
# ──────────────────────────────────────────────────────────────
client = mqtt.Client(
    client_id=f"CsvConsumer_{os.getpid()}",
    clean_session=True
)
client.on_connect    = on_connect
client.on_subscribe  = on_subscribe
client.on_message    = on_message
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