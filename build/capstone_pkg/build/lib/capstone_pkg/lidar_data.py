#! /usr/bin/env python

import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class lidar_data_node(Node):
    def __init__(self):
        super().__init__("lidar_data")

        # qos profile must be qos_profile_sensora_data otherwise data will not be retrieved
        self.laser_data_subscriber = self.create_subscription(
            LaserScan, "scan", self.callback_lidar_scan, qos_profile_sensor_data)
        self.get_logger().info("lidar data node has started")

    def callback_lidar_scan(self, msg):
        data = {0: "{:.4f}".format(msg.ranges[0]), 45: "{:.4f}".format(msg.ranges[45]),
                90: round(msg.ranges[90], 4), 135: round(msg.ranges[135], 4),
                180: round(msg.ranges[180], 4), 225: round(msg.ranges[225], 4),
                270: "{:.4f}".format(msg.ranges[270])}
        print(data)


def main(args=None):
    rclpy.init(args=args)
    node = lidar_data_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
