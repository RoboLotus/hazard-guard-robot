#!/usr/bin/env python3
"""ROS 2 to MQTT bridge for the HazardGuard IoT layer."""

import json
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

import paho.mqtt.client as mqtt


def make_qos(spec):
    if spec == 'sensor':
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
    if spec == 'latched':
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def new_mqtt_client(client_id):
    try:
        return mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
        )
    except AttributeError:
        return mqtt.Client(client_id=client_id)


class MqttBridgeNode(Node):

    def __init__(self):
        super().__init__('mqtt_bridge_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('broker_host', 'localhost')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('base_topic', 'hazardguard')
        self.declare_parameter('robot_id', 'robot01')

        config_file = self.get_parameter('config_file').value
        self.host = self.get_parameter('broker_host').value
        self.port = int(self.get_parameter('broker_port').value)
        self.base = self.get_parameter('base_topic').value
        self.robot_id = self.get_parameter('robot_id').value

        self.prefix = '{}/{}'.format(self.base, self.robot_id)
        self.status_topic = '{}/status'.format(self.prefix)
        self.last_sent = {}

        if not config_file:
            raise RuntimeError('config_file parameter is required')

        with open(config_file, 'r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle) or {}

        self.mappings = config.get('mappings', [])

        self.client = new_mqtt_client('hazard_guard_{}'.format(self.robot_id))
        self.client.will_set(
            self.status_topic,
            json.dumps({'online': False}),
            qos=1,
            retain=True,
        )
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()
        self.client.publish(
            self.status_topic,
            json.dumps({'online': True}),
            qos=1,
            retain=True,
        )

        self.get_logger().info(
            'MQTT broker {}:{}, prefix {}'.format(
                self.host, self.port, self.prefix)
        )

        self.subs = []
        for entry in self.mappings:
            self._add_mapping(entry)

    def _add_mapping(self, entry):
        ros_topic = entry['ros_topic']
        type_name = entry['ros_type']
        mqtt_suffix = entry['mqtt_topic']
        min_interval = float(entry.get('min_interval', 0.0))
        retain = bool(entry.get('retain', False))
        qos_spec = entry.get('qos', 'default')

        try:
            msg_class = get_message(type_name)
        except (ImportError, AttributeError, ValueError) as exc:
            self.get_logger().error(
                'skipping {}: cannot resolve type {} ({})'.format(
                    ros_topic, type_name, exc)
            )
            return

        full_mqtt_topic = '{}/{}'.format(self.prefix, mqtt_suffix)

        def callback(msg, topic=full_mqtt_topic, key=ros_topic,
                     interval=min_interval, do_retain=retain):
            now = time.monotonic()
            if interval > 0.0:
                previous = self.last_sent.get(key, 0.0)
                if now - previous < interval:
                    return
            self.last_sent[key] = now

            payload = {
                'robot_id': self.robot_id,
                'stamp': self.get_clock().now().nanoseconds / 1e9,
                'data': message_to_ordereddict(msg),
            }
            self.client.publish(
                topic,
                json.dumps(payload, ensure_ascii=False, default=str),
                qos=1,
                retain=do_retain,
            )

        self.subs.append(
            self.create_subscription(
                msg_class, ros_topic, callback, make_qos(qos_spec)
            )
        )
        self.get_logger().info('{} -> {}'.format(ros_topic, full_mqtt_topic))

    def shutdown(self):
        try:
            self.client.publish(
                self.status_topic,
                json.dumps({'online': False}),
                qos=1,
                retain=True,
            )
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
