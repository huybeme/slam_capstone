import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from capstone_interfaces.msg import TB3Status


class circle_around_node(Node):
    def __init__(self):
        super().__init__("circle_around")
        
        self.tb_status = TB3Status()
        self.get_lidar_val = self.create_subscription(
            TB3Status, "tb3_lidar_values", self.callback_lidar_and_move, 10)

        self.movement = Twist()
        self.publish_movement = self.create_publisher(
            Twist, "cmd_vel", 10
        )

        self.get_logger().info("circling around has begun")

    def callback_lidar_and_move(self, msg):

        # get robot moving parallel with wall at ~10in apart
        if msg.lidar_data[7] < 0.7 and msg.lidar_data[1] > 0.400:
            # self.movement.angular.z = 0.1 - msg.lidar_data[7]/3.50    # maybe too slow

            self.movement.angular.z = 0.2 - msg.lidar_data[7]/1.75



        #   lidar data is an array of 8 elements
        #       F      R      B      L
        #   NW  N  NE  E  SE  S  SW  W
        #   0   1  2   3  4   5  6   7
        if msg.lidar_data[1] > 1.500: # ~ 5ft
            self.movement.linear.x = 0.150

        elif msg.lidar_data[1] < 0.299 or msg.lidar_data[2] < 0.25:       # 10 in
            self.movement.linear.x = 0.0
            self.movement.angular.z = 0.8

        elif msg.lidar_data[1] < 0.699 and msg.lidar_data[1] > 0.300:        # 10in to 2ft in, very precise
            self.movement.linear.x = 0.100

        elif msg.lidar_data[1] < 1.499 and msg.lidar_data[1] > 0.700:   # 2 to 5 ft, fairly precise
            self.movement.linear.x = 0.125

        
        
        
        self.publish_movement.publish(self.movement)


def main(args=None):
    rclpy.init(args=args)
    node = circle_around_node()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()