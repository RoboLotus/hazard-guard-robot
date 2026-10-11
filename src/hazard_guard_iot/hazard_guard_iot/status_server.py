#!/usr/bin/env python3
"""HTTP status server for the SmartThings Edge Driver."""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import paho.mqtt.client as mqtt

BROKER_HOST = os.environ.get('HG_BROKER', 'localhost')
BROKER_PORT = int(os.environ.get('HG_BROKER_PORT', '1883'))
ROBOT_ID = os.environ.get('HG_ROBOT_ID', 'robot01')
HTTP_PORT = int(os.environ.get('HG_HTTP_PORT', '8088'))

# 3S lithium pack on the ROSMASTER M1. Adjust if the pack is 4S.
VOLTAGE_FULL = float(os.environ.get('HG_VOLTAGE_FULL', '12.6'))
VOLTAGE_EMPTY = float(os.environ.get('HG_VOLTAGE_EMPTY', '9.0'))

TELEMETRY_TOPIC = 'hazardguard/{}/telemetry'.format(ROBOT_ID)
STATUS_TOPIC = 'hazardguard/{}/status'.format(ROBOT_ID)
THERMAL_TOPIC = 'hazardguard/{}/thermal'.format(ROBOT_ID)
BATTERY_TOPIC = 'hazardguard/{}/battery'.format(ROBOT_ID)
SYSTEM_TOPIC = 'hazardguard/{}/system'.format(ROBOT_ID)

_lock = threading.Lock()
_state = {
    'online': False,
    'battery': None,
    'voltage': None,
    'temperature': None,
    'alert': 'clear',
    'mode': 'unknown',
    'network': 'unknown',
    'lidar': 'unknown',
    'cpu_percent': None,
    'cpu_temp': None,
    'memory_percent': None,
    'disk_percent': None,
    'simulated': None,
    'updated_at': 0.0,
}


def on_connect(client, userdata, flags, reason_code, properties=None):
    for topic in (TELEMETRY_TOPIC, STATUS_TOPIC, THERMAL_TOPIC,
                  BATTERY_TOPIC, SYSTEM_TOPIC):
        client.subscribe(topic, qos=1)


def on_message(client, userdata, message):
    try:
        envelope = json.loads(message.payload.decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        return

    with _lock:
        if message.topic == STATUS_TOPIC:
            _state['online'] = bool(envelope.get('online', False))
            _state['updated_at'] = time.time()
            return

        data = envelope.get('data') or {}

        if message.topic == TELEMETRY_TOPIC:
            battery = data.get('battery_percent')
            if isinstance(battery, (int, float)):
                _state['battery'] = int(round(battery))

            temperature = data.get('max_temperature_c')
            if isinstance(temperature, (int, float)):
                _state['temperature'] = round(float(temperature), 1)

            alert = data.get('alert_level')
            if isinstance(alert, str):
                _state['alert'] = alert

            for key, field in (('mode', 'mode'),
                               ('network', 'network_quality'),
                               ('lidar', 'lidar_status')):
                value = data.get(field)
                if isinstance(value, str):
                    _state[key] = value

            if isinstance(data.get('mock'), bool):
                _state['simulated'] = data['mock']

        elif message.topic == THERMAL_TOPIC:
            temperature = data.get('temperature_c')
            if isinstance(temperature, (int, float)):
                _state['temperature'] = round(float(temperature), 1)
            if isinstance(data.get('simulated'), bool):
                _state['simulated'] = data['simulated']

        elif message.topic == SYSTEM_TOPIC:
            # std_msgs/String wraps the payload as a JSON string
            try:
                metrics = json.loads(data.get('data') or '{}')
            except (ValueError, TypeError):
                metrics = {}
            for key in ('cpu_percent', 'memory_percent', 'disk_percent'):
                value = metrics.get(key)
                if isinstance(value, (int, float)):
                    _state[key] = round(float(value), 1)
            cpu_temp = metrics.get('cpu_temp_c')
            if isinstance(cpu_temp, (int, float)):
                _state['cpu_temp'] = round(float(cpu_temp), 1)

        elif message.topic == BATTERY_TOPIC:
            percentage = data.get('percentage')
            voltage = data.get('voltage')

            if isinstance(percentage, (int, float)) and percentage > 0:
                value = percentage * 100 if percentage <= 1.0 else percentage
                _state['battery'] = int(round(value))
            elif isinstance(voltage, (int, float)) and voltage > 0:
                span = VOLTAGE_FULL - VOLTAGE_EMPTY
                ratio = (float(voltage) - VOLTAGE_EMPTY) / span
                ratio = max(0.0, min(1.0, ratio))
                _state['battery'] = int(round(ratio * 100))

            if isinstance(voltage, (int, float)):
                _state['voltage'] = round(float(voltage), 2)

            _state['simulated'] = False

        _state['online'] = True
        _state['updated_at'] = time.time()


def snapshot():
    with _lock:
        payload = dict(_state)
    age = time.time() - payload['updated_at'] if payload['updated_at'] else None
    if age is not None and age > 30.0:
        payload['online'] = False
    payload['stale_seconds'] = round(age, 1) if age is not None else None
    payload['robot_id'] = ROBOT_ID
    return payload


class Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path.split('?')[0].rstrip('/') not in ('/status', ''):
            self.send_response(404)
            self.end_headers()
            return

        body = json.dumps(snapshot()).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


def new_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def main():
    client = new_client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
    client.loop_start()

    server = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print('status server on 0.0.0.0:{}/status'.format(HTTP_PORT))
    print('battery curve: {}V empty, {}V full'.format(
        VOLTAGE_EMPTY, VOLTAGE_FULL))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        client.loop_stop()
        client.disconnect()


if __name__ == '__main__':
    main()
