import rclpy
# need this buffer for lidar sensor
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float32MultiArray

from capstone_interfaces.msg import TB3Status
from capstone_interfaces.msg import TB3Link
import capstone_pkg.capstone_function as capstone_function

import os


class tb3_status_node(Node):
    def __init__(self):
        super().__init__("tb3_status")

        # setup for transform listening
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(1, self.on_tf_timer)
        self.odom_str_parent = 'odom'
        self.basefp_str_child = 'base_footprint'

        self.odom_bfp_tf_trans_x = None
        self.odom_bfp_tf_trans_y = None

        # setup for topics
        self.battery_percent_sub = self.create_subscription(
            BatteryState, "battery_state", self.callback_battery, 10
        )

        self.battery = 0

        self.lidar_sub = self.create_subscription(
            LaserScan, "scan", self.callback_tb3_status, qos_profile_sensor_data
        )

        self.tb3_status_pub = self.create_publisher(
            TB3Status, "tb3_status", 10
        )

        self.tmp_sub = self.create_subscription(
            Float32, "tmp_node", self.callback_tmp, qos_profile_sensor_data
        )

        self.link_status = TB3Link()
        self.link_sub = self.create_subscription(
            TB3Link, "tb3_link", self.callback_link_sub, qos_profile_sensor_data
        )

        self.tmp = 0

        self.node_status_check()

        self.cartographer_ready = False

        self.get_logger().info("tb3_status_node has been started")

    def node_status_check(self):
        circle_node = capstone_function.send_service_request(self, "circle_around_check", "circle_around")
        self.get_logger().info(str(circle_node))

        world_node = capstone_function.send_service_request(self, "robot_world_check", "robot_world")
        self.get_logger().info(str(world_node))

    def callback_link_sub(self, msg):
        self.link_status = msg

    def on_tf_timer(self):
        trans = None
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.odom_str_parent, self.basefp_str_child, now
            )

            self.odom_bfp_tf_trans_x = trans.transform.translation.x
            self.odom_bfp_tf_trans_y = trans.transform.translation.y

            # if abs(self.odom_bf_trans_x) <= 0.05 and abs(self.odom_bf_trans_y) <= 0.05 and not self.cartographer_ready:
            #     self.cartographer_ready = True
            #     # os.system("ros2 launch turtlebot3_cartographer cartographer.launch.py")
            #     self.get_logger().info("robot position within resolution, cartographer package launched")

        except:
            self.get_logger().info("tf listeners not listening")
            return

    def callback_battery(self, msg):
        self.battery = msg.percentage

    def callback_tmp(self, msg):
        self.tmp = msg.data

    def callback_tb3_status(self, msg):
        lidar_arr = [float("{:.3f}".format(msg.ranges[45])),    float("{:.3f}".format(msg.ranges[0])),      float("{:.3f}".format(msg.ranges[315])),
                     float("{:.3f}".format(msg.ranges[90])),                                                float(
                         "{:.3f}".format(msg.ranges[270])),
                     float("{:.3f}".format(msg.ranges[135])),   float("{:.3f}".format(msg.ranges[180])),    float("{:.3f}".format(msg.ranges[225]))]

        # note, sensor installed backwards
        lidar_front = {'315': lidar_arr[0],
                       '__0': lidar_arr[1], '_45': lidar_arr[2]}
        lidar_sides = {'270': lidar_arr[3],
                       'XXX': "XXXXX", '_90': lidar_arr[4]}
        lidar_rear = {'225': lidar_arr[5],
                      '180': lidar_arr[6], '135': lidar_arr[7]}

        # print out some information for troubleshooting purposes
        status_msg = TB3Status()
        logger = "\n" + str(lidar_front) + "\n" + str(lidar_sides) + "\n" + str(lidar_rear) \
            + "\n" + "battery: {:.2f}".format(self.battery) + "\ttemperature: {:.2f}".format(self.tmp) \
            + "\n" + \
            f"parent:  {self.odom_str_parent}\t\t child: {self.basefp_str_child}"

        if self.odom_bfp_tf_trans_x is None or self.odom_bfp_tf_trans_y is None:
            self.get_logger().info(logger + "\nlistener not ready yet" + "\n")
        else:
            status_msg.robot_pos_x = self.odom_bfp_tf_trans_x
            status_msg.robot_pos_y = self.odom_bfp_tf_trans_y
            self.get_logger().info(logger +
                                   "\nx: {:.2e}".format(self.odom_bfp_tf_trans_x) + "\t\ty: {:.2e}".format(self.odom_bfp_tf_trans_y) + "\n")

        self.get_logger().info(str(self.link_status))

        # publish stuff as needed
        status_msg.lidar_data = lidar_arr
        self.tb3_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = tb3_status_node()

    # we will be stuck here until keyboard stop
    rclpy.spin(node)

    # stop movement of robot when spin is stopped
    publish_stop_movement = node.create_publisher(Twist, 'cmd_vel', 10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    publish_stop_movement.publish(twist)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
