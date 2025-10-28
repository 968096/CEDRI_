#!/usr/bin/env python3
import os
import base64
import signal
import logging
import csv
import json
import paho.mqtt.client as mqtt
from measurement_pb2 import SensorGpsReading  # gerado do seu .proto

BROKER   = "broker.emqx.io"
PORT     = 1883
TOPIC    = "application/test/caio/bme688"
OUT_FILE = "data16steps/bme68816_coffeeandoilmixed_.csv"
QOS      = 1

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mqtt-bme-consumer")

# Colunas do CSV (sem iso_time e sem nome do heater profile)
CSV_FIELDS = [
    "device_id","location_id","sensor_id","heater_profile",
    "measurement_step","heater_setpoint_c","heater_duration_ms",
    "temp_c","humidity_pct","pressure_hpa",
    "gas_resistance_ohm","gas_valid","heat_stable",
    "timestamp","latitude","longitude","volume_l",
    "total_steps","step_window_ms","cycle_period_ms","cycle_index"
]

def _ensure_header_once():
    """Escreve o header apenas se o arquivo não existir ou estiver vazio
       ou se a primeira linha não bater com o cabeçalho esperado."""
    need_header = True
    if os.path.isfile(OUT_FILE) and os.path.getsize(OUT_FILE) > 0:
        try:
            with open(OUT_FILE, "r", encoding="utf-8") as f:
                first_line = f.readline().strip()
                expected = ",".join(CSV_FIELDS)
                need_header = (first_line != expected)
        except Exception:
            need_header = True
    if need_header:
        with open(OUT_FILE, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()

_ensure_header_once()

# ---- MQTT (API v2 para evitar DeprecationWarning) ----
def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        logger.info(f"✅ Connected to {BROKER}:{PORT}")
        client.subscribe(TOPIC, qos=QOS)
        logger.info(f"📡 Subscribed: {TOPIC}")
    else:
        logger.error(f"❌ Connect failed: rc={reason_code}")

def on_message(client, userdata, msg):
    try:
        payload_json = json.loads(msg.payload.decode("utf-8"))
        b64_payload = payload_json.get("data")
        if not b64_payload:
            return

        raw = base64.b64decode(b64_payload)

        reading = SensorGpsReading()
        reading.ParseFromString(raw)

        # Monta a linha (heater_profile = ID numérico; sem iso_time)
        row = {
            "device_id":           reading.device_id,
            "location_id":         reading.location_id,
            "sensor_id":           reading.sensor_id,
            "heater_profile":      int(reading.heater_profile),
            "measurement_step":    reading.measurement_step,

            # << os "step heater" que você quer >>
            "heater_setpoint_c":   reading.heater_setpoint_c,
            "heater_duration_ms":  reading.heater_duration_ms,

            "temp_c":              reading.temp_c,
            "humidity_pct":        reading.humidity_pct,
            "pressure_hpa":        reading.pressure_hpa,
            "gas_resistance_ohm":  reading.gas_resistance_ohm,
            "gas_valid":           int(reading.gas_valid),
            "heat_stable":         int(reading.heat_stable),

            "timestamp":           reading.timestamp,
            "latitude":            reading.latitude,
            "longitude":           reading.longitude,
            "volume_l":            reading.volume_l,

            # Metadados do ciclo
            "total_steps":         reading.total_steps,
            "step_window_ms":      reading.step_window_ms,
            "cycle_period_ms":     reading.cycle_period_ms,
            "cycle_index":         reading.cycle_index,
        }

        with open(OUT_FILE, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
            writer.writerow(row)

        logger.info(
            "CSV ← hp=%s step=%s set=%.1f°C dur=%dms gas=%s t=%.2f h=%.2f p=%.2f",
            row["heater_profile"], row["measurement_step"],
            row["heater_setpoint_c"], row["heater_duration_ms"],
            row["gas_resistance_ohm"], row["temp_c"], row["humidity_pct"], row["pressure_hpa"]
        )

    except Exception as e:
        logger.error(f"❌ on_message error: {e}")

def _shutdown(sig, frame):
    logger.info("🛑 Gracefully shutting down…")
    try:
        client.disconnect()
    finally:
        raise SystemExit

signal.signal(signal.SIGINT, _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"protobuf_consumer_{os.getpid()}")
client.on_connect = on_connect
client.on_message = on_message

logger.info("🔌 Connecting…")
client.connect(BROKER, PORT, keepalive=60)
client.loop_forever()
