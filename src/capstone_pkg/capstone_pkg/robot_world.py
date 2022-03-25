import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import time


class RobotWordNode(Node):
    def __init__(self):
        super().__init__("blahblah")

        self.sub_map = self.create_subscription(
            OccupancyGrid, "global_map", self.get_grid, 10
        )

        # self.sub_odom = self.create_subscription(
        #     Odometry, "odom", self.get_odom, 10
        # )

    def get_odom(self, msg):
        print(msg.pose.pose.position)
        print(msg.pose.pose.orientation)
        # print(msg.twist.twist.linear)
        # print(msg.twist.twist.angular)

    def get_grid(self, msg):
        pass

        with open('local_grid.txt', 'w') as output_grid:
            for i, grid in enumerate(msg.data):
                if grid == 0:
                    output_grid.write(' ')
                elif grid == -1:
                    output_grid.write('*')
                else:
                    output_grid.write("o")

                if i % msg.info.width == 0 and i > 0:
                    output_grid.write('\n')


        # self.print_local_grid(msg.data)

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
