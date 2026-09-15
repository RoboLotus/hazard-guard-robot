#!/usr/bin/env python3
"""Publishes Jetson system metrics as JSON on /hazard_guard/system."""

import json
import os
import shutil

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

THERMAL_ROOT = '/sys/devices/virtual/thermal'


def read_thermal_zones():
    zones = {}
    for index in range(16):
        base = os.path.join(THERMAL_ROOT, 'thermal_zone{}'.format(index))
        if not os.path.isdir(base):
            break
        try:
            with open(os.path.join(base, 'type')) as handle:
                name = handle.read().strip()
            with open(os.path.join(base, 'temp')) as handle:
                milli = int(handle.read().strip())
        except (OSError, ValueError, TypeError):
            continue
        zones[name] = round(milli / 1000.0, 1)
    return zones


def read_memory():
    values = {}
    try:
        with open('/proc/meminfo') as handle:
            for line in handle:
                key, _, rest = line.partition(':')
                values[key] = int(rest.split()[0])
    except (OSError, ValueError, IndexError):
        return None, None, None

    total = values.get('MemTotal')
    available = values.get('MemAvailable')
    if not total or available is None:
        return None, None, None

    used = total - available
    return (
        round(used / 1024.0),
        round(total / 1024.0),
        round(used * 100.0 / total, 1),
    )


def read_disk(path='/'):
    try:
        usage = shutil.disk_usage(path)
    except OSError:
        return None, None, None
    return (
        round(usage.used / 1024 ** 3, 1),
        round(usage.total / 1024 ** 3, 1),
        round(usage.used * 100.0 / usage.total, 1),
    )


class CpuSampler:

    def __init__(self):
        self.previous = None

    def read(self):
        try:
            with open('/proc/stat') as handle:
                fields = handle.readline().split()
        except OSError:
            return None
        if len(fields) < 5 or fields[0] != 'cpu':
            return None

        numbers = [int(value) for value in fields[1:]]
        idle = numbers[3] + (numbers[4] if len(numbers) > 4 else 0)
        total = sum(numbers)

        if self.previous is None:
            self.previous = (idle, total)
            return None

        prev_idle, prev_total = self.previous
        self.previous = (idle, total)

        delta_total = total - prev_total
        if delta_total <= 0:
            return None

        busy = delta_total - (idle - prev_idle)
        return round(busy * 100.0 / delta_total, 1)


class SystemMonitorNode(Node):

    def __init__(self):
        super().__init__('hazard_guard_system_monitor')
        self.declare_parameter('publish_period', 5.0)
        period = float(self.get_parameter('publish_period').value)

        self._cpu = CpuSampler()
        self._cpu.read()
        self._publisher = self.create_publisher(
            String, '/hazard_guard/system', 10)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            'publishing system metrics every {}s'.format(period))

    def _publish(self):
        zones = read_thermal_zones()
        mem_used, mem_total, mem_percent = read_memory()
        disk_used, disk_total, disk_percent = read_disk()

        payload = {
            'cpu_percent': self._cpu.read(),
            'cpu_temp_c': zones.get('cpu-thermal'),
            'gpu_temp_c': zones.get('gpu-thermal'),
            'junction_temp_c': zones.get('tj-thermal'),
            'memory_used_mb': mem_used,
            'memory_total_mb': mem_total,
            'memory_percent': mem_percent,
            'disk_used_gb': disk_used,
            'disk_total_gb': disk_total,
            'disk_percent': disk_percent,
        }

        message = String()
        message.data = json.dumps(payload)
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
