import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import time


class RobotWordNode(Node):
    def __init__(self):
        super().__init__("blahblah")

        self.sub_map = self.create_subscription(
            OccupancyGrid, "map", self.get_grid, 10
        )

    def get_grid(self, msg):
        self.print_local_grid(msg.data)

    def print_local_grid(self, msg):
        for i, cell in enumerate(msg):
            print(i, cell, self.msg[i])
            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = RobotWordNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
