import rclpy
from rclpy.qos import qos_profile_sensor_data # need this buffer for lidar sensor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from capstone_interfaces.msg import TB3Status


class tb3_status_node(Node):
    def __init__(self):
        super().__init__("tb3_status")

        self.battery_percent_sub = self.create_subscription(
            BatteryState, "battery_state", self.callback_battery, 10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan, "scan", self.callback_lidar, qos_profile_sensor_data
        )

        self.lidar_pub = self.create_publisher(
            TB3Status, "tb3_lidar_values", 10
        )

        print("tb3_status_node has been started")

    def callback_battery(self, msg):
        if msg.percentage < 20.0 and msg.percentage > 0.1:
            print("low battery: (%): {:.2f}".format(msg.percentage))

    def callback_lidar(self, msg):
        lidar_arr = [float("{:.3f}".format(msg.ranges[225])), float("{:.3f}".format(msg.ranges[180])), float("{:.3f}".format(msg.ranges[135])),
                     float("{:.3f}".format(msg.ranges[270])), float("{:.3f}".format(msg.ranges[90])),
                     float("{:.3f}".format(msg.ranges[315])), float("{:.3f}".format(msg.ranges[0])), float("{:.3f}".format(msg.ranges[45]))]
                     
        # note, sensor installed backwards
        lidar_front = {'315': lidar_arr[0], '__0': lidar_arr[1], '_45': lidar_arr[2]}
        lidar_sides = {'270': lidar_arr[3], 'XXX': "XXXXX", '_90': lidar_arr[4]}
        lidar_rear = {'225': lidar_arr[5], '180': lidar_arr[6], '135': lidar_arr[7]}

        print(lidar_front)
        print(lidar_sides)
        print(lidar_rear)
        print()
        lidar_msg = TB3Status()
        lidar_msg.lidar_data = lidar_arr
        self.lidar_pub.publish(lidar_msg)


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
