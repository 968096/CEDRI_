#!/usr/bin/env python3
import json
import time
import paho.mqtt.client as mqtt
from datetime import datetime

# MQTT settings
BROKER = "broker.emqx.io"
PORT = 1883
TOPIC = "home/sensors/bme688_hierarchical"
CLIENT_ID = f"hierarchical_consumer_{int(time.time())}"

# Output file
JSON_FILE = "bme688_hierarchical_data.json"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe(TOPIC, qos=0)
        print(f"Subscribed to topic: {TOPIC}")
    else:
        print(f"Connect failed with return code {rc}")

def on_message(client, userdata, msg):
    try:
        # Parse incoming JSON
        data = json.loads(msg.payload.decode('utf-8'))

        # Add reception metadata
        data['received_at'] = datetime.now().isoformat()
        data['message_size_bytes'] = len(msg.payload)

        # Append to JSON file
        with open(JSON_FILE, 'a') as f:
            json.dump(data, f, indent=2)
            f.write('\n')
            f.write('='*50 + '\n')

        # Extract summary info for console
        device = data.get('device', {})
        device_id = device.get('device_id', 'Unknown')
        sensors = device.get('sensors', [])

        total_measurements = sum(len(s.get('measurements', [])) for s in sensors)

        print(f"Message received:")
        print(f"  Device: {device_id}")
        print(f"  Sensors: {len(sensors)}")
        print(f"  Total measurements: {total_measurements}")
        print(f"  Message size: {len(msg.payload)} bytes")
        print(f"  Saved to: {JSON_FILE}")
        print("-" * 40)

    except json.JSONDecodeError as e:
        print(f"JSON parse error: {e}")
    except Exception as e:
        print(f"Error processing message: {e}")

def main():
    # Create new JSON file with header
    with open(JSON_FILE, 'w') as f:
        f.write(f"# BME688 Hierarchical Data Log\n")
        f.write(f"# Started: {datetime.now().isoformat()}\n")
        f.write(f"# Format: One JSON object per message\n")
        f.write("="*50 + "\n")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {BROKER}:{PORT}")
    client.connect(BROKER, PORT, keepalive=60)

    print("Listening for BME688 sensor data...")
    print(f"Data will be saved to: {JSON_FILE}")
    print("Press Ctrl+C to stop")
    print("="*50)

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print(f"\nConsumer stopped by user")
        print(f"All data saved in: {JSON_FILE}")
        client.disconnect()

if __name__ == "__main__":
    main()