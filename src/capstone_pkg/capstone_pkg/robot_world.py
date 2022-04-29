from concurrent.futures import process
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Vector3, Quaternion
from std_srvs.srv import SetBool
import numpy as np

AREA = [
    [-1, 2], [0, 2], [1, 2],
    [-2, 1], [-1, 1], [0, 1], [1, 1], [2, 1],
    [-2, 0], [-1, 0], [0, 0], [1, 0], [2, 0],
    [-2, -1], [-1, -1], [0, -1], [1, -1], [2, -1],
    [-1, -2], [0, -2], [1, -2]
]


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
        self.timer = self.create_timer(1.0, self.on_timer)

        self.vector = Vector3()
        self.q = Quaternion()

        # will get a bool from circle_around node
        self.robot_init_service = self.create_service(
            SetBool, "robot_initialization", self.callback_service
        )
        self.robot_service_completed = False

        self.start_found = False
        self.start_xy_pose = None
        self.start_xy_grid = None
        self.start_i = None

        # do i need this init
        self.robot_area_pose = []
        self.robot_area_grid = []
        self.robot_area_i = []

        self.get_logger().info("robot world started")

    def callback_service(self, request, response):
        req = request.data
        if req:
            self.robot_service_completed = req
            self.get_logger().info("robot initialization service completed")
            response.success = True
            response.message = "got your message"
            return response
        else:
            response.success = False
            response.message = "something happened?"
            return response

    # get transforms - these frames will provide robot poses relative to map frame
    def on_timer(self):
        from_frame = self.target_frame
        to_frame = 'map'
        try:
            now = self.get_clock().now().to_msg()
            # go back in time n sec to get last frame
            now.nanosec = now.nanosec - 200000000
            trans = self.buffer.lookup_transform(
                to_frame,
                from_frame,
                now
            )
        except:
            print("could not find transform between map>base_fp")
            return
        self.vector = trans.transform.translation
        self.q = trans.transform.rotation

    def get_grid(self, msg):

        # get robot pose for getting coordinates and then map index from coordinates
        robot_xy = self.world_to_grid(
            self.vector.x, self.vector.y, msg.info.origin.position.x,
            msg.info.origin.position.y, msg.info.resolution, msg.info.width, msg.info.height
        )
        robot_i = self.to_index(robot_xy[0], robot_xy[1], msg.info.width)

        # get location after robot initialzation through services
        if self.robot_service_completed:
            self.start_xy_pose = [self.vector.x, self.vector.y]  # robot_xy
            self.start_found = True

        start_area_grid = []
        start_area_i = []
        if self.start_found:
            self.start_xy_grid = self.world_to_grid(
                self.start_xy_pose[0], self.start_xy_pose[1], msg.info.origin.position.x,
                msg.info.origin.position.y, msg.info.resolution, msg.info.width, msg.info.height
            )
            self.start_i = self.to_index(
                self.start_xy_grid[0], self.start_xy_grid[1], msg.info.width)

            for area in AREA:
                temp = np.add(area, self.start_xy_grid)
                start_area_grid.append(temp.tolist())

            for grid in start_area_grid:
                start_area_i.append(self.to_index(
                    grid[0], grid[1], msg.info.width))
        else:
            return

        map_origin_xy = self.world_to_grid(
            0.0, 0.0, msg.info.origin.position.x,
            msg.info.origin.position.y, msg.info.resolution, msg.info.width, msg.info.height
        )
        map_origin_i = self.to_index(
            map_origin_xy[0], map_origin_xy[1], msg.info.width)

        # outputting to txt file is for troubleshooting
        with open('local_grid.txt', 'w') as output:
            for i, grid in enumerate(msg.data, 1):
                if i == robot_i:
                    output.write(" [r] ")
                elif i == map_origin_i:
                    output.write(" [m] ")
                elif (i in start_area_i):
                    output.write(" [s] ")
                elif grid == -1:
                    output.write("  *  ")
                elif grid >= 0 and grid < 10:
                    output.write("  " + str(grid) + "  ")
                elif grid >= 10 and grid < 100:
                    output.write("  " + str(grid) + " ")
                elif grid == 100:
                    output.write(" " + str(grid) + " ")
                else:
                    output.write(grid)

                if i % msg.info.width == 0 and i > 0:
                    output.write("\n")

    # do i use this?

    def update_start(self, xy, resolution, origin_x, origin_y):
        gx = xy[0]
        gy = xy[1]

        x = gx * resolution + origin_x
        y = gy * resolution + origin_y

        return [x, y]

    def to_index(self, x, y, width):
        return (y * width + x)

    def world_to_grid(self, x, y, origin_x, origin_y, resolution, size_x, size_y):
        # if outside of grid
        if (x < origin_x or x >= resolution * size_x) or \
                (y < origin_y or y >= size_y * size_y):
            return [1, 1]

        gx = int((x - origin_x) / resolution)
        gy = int((y - origin_y) / resolution)

        # print(x, y, gx, gy)
        return [gx, gy]


def main(args=None):
    rclpy.init(args=args)
    node = RobotWordNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
