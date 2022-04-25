import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, TypeException
from example_interfaces.srv import SetBool
import time


class RobotWordNode(Node):
    def __init__(self):
        super().__init__("robot_world")

        self.sub_map = self.create_subscription(
            OccupancyGrid, "map", self.get_grid, 10
        )

        self.declare_parameter("target_frame", "base_footprint")
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # self.timer = self.create_timer(1.0, self.on_timer)

        self.get_logger().info("robot world started")

    def on_timer(self):
        from_frame = self.target_frame
        to_frame = 'map'
        try:
            now = self.get_clock().now().to_msg()
            # go back in time 0.1 sec to get last frame
            now.nanosec = now.nanosec - 100000000
            trans = self.buffer.lookup_transform(
                to_frame,
                from_frame,
                now
            )
        except TypeException as e:
            print("could not find transform", e)
            return
        print(trans.transform.translation)
        print(trans.transform.rotation)

    def get_grid(self, msg):

        with open('local_grid.txt', 'w') as output_grid:
            for i, grid in enumerate(msg.data):

                if grid == -1:
                    output_grid.write(" * ")
                elif grid >= 0 and grid < 10:
                    output_grid.write(" " + str(grid) + " ")
                elif grid >= 10 and grid < 100:
                    output_grid.write(str(grid) + " ")
                elif grid == 100:
                    output_grid.write(str(grid))
                else:
                    output_grid.write(" ")

                if i % msg.info.width == 0 and i > 0:
                    output_grid.write('\n')

    # for i in range(len(msg.data) -1, -1, -1): enumerate array backwards
    


def main(args=None):
    rclpy.init(args=args)
    node = RobotWordNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
