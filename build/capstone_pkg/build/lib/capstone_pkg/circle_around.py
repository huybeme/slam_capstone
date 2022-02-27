import rclpy
from rclpy.node import Node

from capstone_interfaces.msg import TB3Status


class circle_around_node(Node):
    def __init__(self):
        super().__init__("circle_around")
        
        self.get_lidar_val = self.create_subscription(TB3Status, "tb3_lidar_values", self.callback_lidar_and_move, 10)


        self.get_logger().info("circling around has begun")

    def callback_lidar_and_move(self, msg):
        print(msg.lidar_data)


def main(args=None):
    rclpy.init(args=args)
    node = circle_around_node()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()