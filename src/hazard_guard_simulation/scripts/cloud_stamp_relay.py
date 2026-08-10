#!/usr/bin/env python3
"""Align RGB-D clouds with the latest available physical odometry TF.

The HP60C driver timestamps images a few seconds ahead of the M1 odometry
TF.  ``point_cloud_assembler`` then cannot transform a cloud at its original
timestamp and discards the whole accumulated cloud.  A zero ROS timestamp
means "latest transform" to tf2, which is the appropriate choice for this
live visualization stream.
"""

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudStampRelay(Node):
    def __init__(self):
        super().__init__("hazard_guard_cloud_stamp_relay")
        self._publisher = self.create_publisher(
            PointCloud2, "output", qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "input", self._on_cloud, qos_profile_sensor_data
        )

    def _on_cloud(self, message):
        cloud = PointCloud2()
        cloud.header.frame_id = message.header.frame_id
        cloud.header.stamp = Time()
        cloud.height = message.height
        cloud.width = message.width
        cloud.fields = message.fields
        cloud.is_bigendian = message.is_bigendian
        cloud.point_step = message.point_step
        cloud.row_step = message.row_step
        cloud.data = message.data
        cloud.is_dense = message.is_dense
        self._publisher.publish(cloud)


def main():
    rclpy.init()
    node = CloudStampRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
