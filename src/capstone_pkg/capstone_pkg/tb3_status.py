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


class tb3_status_node(Node):
    def __init__(self):
        super().__init__("tb3_status")

        self.declare_parameter('target_frame', 'base_footprint')  # target frame is child
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.parent_frame = 'odom'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.listener_timer = self.create_timer(1, self.on_listener_timer)

        self.trans_x = None
        self.trans_y = None

        self.battery_percent_sub = self.create_subscription(
            BatteryState, "battery_state", self.callback_battery, 10
        )

        self.battery = 0

        self.lidar_sub = self.create_subscription(
            LaserScan, "scan", self.callback_lidar, qos_profile_sensor_data
        )

        self.lidar_pub = self.create_publisher(
            TB3Status, "tb3_lidar_values", 10
        )

        self.tmp_sub = self.create_subscription(
            Float32, "tmp_node", self.callback_tmp, qos_profile_sensor_data
        )

        self.tmp = 0

        print("tb3_status_node has been started")

    def on_listener_timer(self):
        from_frame = self.target_frame  # child
        to_frame = self.parent_frame    # parent
        trans = None

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(to_frame, from_frame, now)

            self.trans_x = trans.transform.translation.x
            self.trans_y = trans.transform.translation.y
        except:
            print("cannot listen to transform")
            return

    def callback_battery(self, msg):
        # print("low battery: (%): {:.2f}".format(msg.percentage))
        self.battery = msg.percentage

    def callback_tmp(self, msg):
        self.tmp = msg.data

    def callback_lidar(self, msg):
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

        print(lidar_front)
        print(lidar_sides)
        print(lidar_rear)
        print("battery: {:.2f}".format(self.battery), "\ttemperature: {:.2f}".format(self.tmp))
        print(f"parent: {self.parent_frame} \t\t child: {self.target_frame}")
        
        if self.trans_x is None or self.trans_y is None:
            print("listener not ready yet")
        else:
            print("x: {:.2e}".format(self.trans_x), "\t\ty: {:.2e}".format(self.trans_y))
        
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
