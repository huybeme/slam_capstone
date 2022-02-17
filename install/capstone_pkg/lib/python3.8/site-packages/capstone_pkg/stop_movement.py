import os
import rclpy

from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile


def main():

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('basic_movement')
    publish_movement = node.create_publisher(Twist, 'cmd_vel', qos)

    twist = Twist()

    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    publish_movement.publish(twist)
