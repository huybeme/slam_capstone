import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist


class tb3_status_node(Node):
    def __init__(self):
        super().__init__("tb3_status")

        self.lidar_sub = self.create_subscription(
            LaserScan, "scan", self.callback_lidar, qos_profile_sensor_data
        )

        self.battery_percent_sub = self.create_subscription(
            BatteryState, "battery_state", self.callback_battery, 10
        )

        print("tb3_status_node has been started")

    def callback_battery(self, msg):
        print(msg.percentage)

    def callback_lidar(self, msg):
        data = {0: "{:.4f}".format(msg.ranges[0]), 45: "{:.4f}".format(msg.ranges[45]),
                90: "{:.4f}".format(msg.ranges[90]), 135: "{:.4f}".format(msg.ranges[135]),
                180: "{:.4f}".format(msg.ranges[180]), 225: "{:.4f}".format(msg.ranges[225]),
                270: "{:.4f}".format(msg.ranges[270])}
        print(data)


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
