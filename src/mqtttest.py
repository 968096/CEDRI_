#!/usr/bin/env python3
import os
import base64
import signal
import logging
import csv
import json
from datetime import datetime
import paho.mqtt.client as mqtt
from measurement_pb2 import SensorGpsReading  # <-- Importa a mensagem única

BROKER = "172.31.255.254"
PORT = 1883
TOPIC = "application/3458bf8d-a319-4085-88cf-ca9cc9e11e6c/device/+/event/up"
OUT_FILE  = "bme688_lora_data1.csv"
QOS       = 1

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mqtt-lora-consumer")

CSV_FIELDS = [
    "device_id", "location_id", "sensor_id", "heater_profile",
    "measurement_step", "temp_c", "humidity_pct", "pressure_hpa",
    "gas_resistance_ohm", "gas_valid", "heat_stable", "timestamp",
    "latitude", "longitude", "satellites"
]

# Cria o CSV se não existir
if not os.path.isfile(OUT_FILE) or os.path.getsize(OUT_FILE) == 0:
    with open(OUT_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_FIELDS)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info(f"✅ Connected to MQTT broker at {BROKER}:{PORT}")
        client.subscribe(TOPIC, qos=QOS)
        logger.info(f"📡 Subscribed to topic: {TOPIC}")
    else:
        logger.error("❌ Connection failed, return code %d", rc)

def on_message(client, userdata, msg):
    try:
        payload_json = json.loads(msg.payload.decode("utf-8"))
        b64_payload = payload_json.get("data")
        if not b64_payload:
            return

        raw = base64.b64decode(b64_payload)

        # Parse como SensorGpsReading
        reading = SensorGpsReading()
        try:
            reading.ParseFromString(raw)
            # Só grava se device_id e GPS válidos
            if reading.device_id and (abs(reading.latitude) > 0.00001 or abs(reading.longitude) > 0.00001):
                row = [
                    reading.device_id,
                    reading.location_id,
                    reading.sensor_id,
                    reading.heater_profile,
                    reading.measurement_step,
                    reading.temp_c,
                    reading.humidity_pct,
                    reading.pressure_hpa,
                    reading.gas_resistance_ohm,
                    int(reading.gas_valid),
                    int(reading.heat_stable),
                    reading.timestamp,
                    reading.latitude,
                    reading.longitude,
                    reading.satellites
                ]
                with open(OUT_FILE, "a", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow(row)
                print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] CSV: {','.join(map(str, row))}")
                return
        except Exception as e:
            pass

    except Exception as e:
        logger.error(f"❌ Failed to decode message: {e}")

def _shutdown(sig, frame):
    logger.info("🛑 Gracefully shutting down…")
    client.disconnect()
    exit(0)

signal.signal(signal.SIGINT, _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

client = mqtt.Client(client_id=f"protobuf_consumer_{os.getpid()}")
client.on_connect = on_connect
client.on_message = on_message

logger.info("🔌 Connecting to broker…")
client.connect(BROKER, PORT, keepalive=60)
client.loop_forever()