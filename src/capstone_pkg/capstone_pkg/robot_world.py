import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import time


class RobotWordNode(Node):
    def __init__(self):
        super().__init__("blahblah")

        self.sub_map = self.create_subscription(
            OccupancyGrid, "map", self.get_grid, 10
        )

        self.sub_odom = self.create_subscription(
            Odometry, "odom", self.get_odom, 10
        )

    def get_odom(self, msg):
        print(msg.pose.pose.position)
        print(msg.pose.pose.orientation)
        # print(msg.twist.twist.linear)
        # print(msg.twist.twist.angular)

    def get_grid(self, msg):
        self.print_local_grid(msg.data)

    def print_local_grid(self, msg):
        path = []
        for i, cell in enumerate(msg):
            if cell >= 0 and cell <= 100:
                path.append([i, cell])
        print(len(path), len(msg),  path)
        print()


def main(args=None):
    rclpy.init(args=args)
    node = RobotWordNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
