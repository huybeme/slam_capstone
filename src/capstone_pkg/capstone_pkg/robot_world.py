from concurrent.futures import process
import rclpy
import os
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Vector3, Quaternion
from std_srvs.srv import SetBool
import numpy as np

from capstone_interfaces.msg import TB3Link
from capstone_interfaces.msg import TB3Tracker
from ament_index_python.packages import get_package_share_directory

AREA = [
    [-1, 2], [0, 2], [1, 2],
    [-2, 1], [-1, 1], [0, 1], [1, 1], [2, 1],
    [-2, 0], [-1, 0], [0, 0], [1, 0], [2, 0],
    [-2, -1], [-1, -1], [0, -1], [1, -1], [2, -1],
    [-1, -2], [0, -2], [1, -2]
]

path = os.path.dirname(__file__)
root = os.path.abspath(os.path.join(path, "..", "..", ".."))
save_maps = os.path.join(root, "maps")


class RobotWorldNode(Node):
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
        self.robot_world_service_check = self.create_service(
            SetBool, "robot_world_check", self.callback_service_check
        )
        self.robot_service_completed = False         # change this back to False

        self.start_found = False
        self.start_xy_pose = None
        self.start_xy_grid = None
        self.start_i = None

        self.RT_map_pos_x = None
        self.RT_map_pos_y = None

        self.tracker_publisher = self.create_publisher(
            TB3Tracker, "tb3_tracker", 10
        )

        # should I track the robot as an area within a world rather than a point
        self.robot_area_pose = []
        self.robot_area_grid = []
        self.robot_area_i = []

        self.state = 0

        self.get_logger().info("robot world started")

    def callback_service_check(self, request, response):
        # wait for robot to finish initialzation and then trigger to take robot cords
        req = request.data
        if req:
            self.robot_service_completed = req
            response.success = True
            response.message = "node ready, client request completed for robot world"
            return response
        else:
            response.success = False
            response.message = "function sent False as request data"
            return response

    # get transforms - these frames will provide robot poses relative to map frame
    def on_timer(self):

        from_frame = self.target_frame
        to_frame = 'map'
        # try to find map > odom transform
        try:
            now = self.get_clock().now().to_msg()
            # go back in time n sec to get last frame since current frame "will not exist"
            now.nanosec = now.nanosec - 200000000
            trans = self.buffer.lookup_transform(
                to_frame,
                from_frame,
                now
            )

        except:
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
        if self.robot_service_completed and self.state == 0:
            self.start_xy_pose = [self.vector.x, self.vector.y]  # robot_xy
            self.start_found = True
            self.state = 1

        # get the area of the initial pose as an array and continually update
        start_area_grid = []    # contains cordinates in grid
        start_area_i = []       # contains indexes of the cordinates in the array
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

        # these map locations are exactly when cartographer is launched
        map_origin_xy = self.world_to_grid(
            0.0, 0.0, msg.info.origin.position.x,
            msg.info.origin.position.y, msg.info.resolution, msg.info.width, msg.info.height
        )
        map_origin_i = self.to_index(
            map_origin_xy[0], map_origin_xy[1], msg.info.width)

        # print(map_origin_xy)            # 2d array coordinates of map
        # print(msg.info.origin.position.x, msg.info.origin.position.y)   # grid coordinates of map
        # self.get_logger().info(str(self.start_xy_pose))       # sometimes this variable is rounded to 0.0.... why
        # self.get_logger().info(str(self.vector.x) + ", " + str(self.vector.y))

        # outputting to txt file is for troubleshooting
        grid_file = "local_grid.txt"
        grid_file_path = os.path.join(save_maps, grid_file)

        with open(grid_file_path, 'w') as output:
            for i, grid in enumerate(msg.data, 1):
                if i == robot_i and i == map_origin_i:
                    output.write(" [X] ")
                elif i == robot_i:
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

        map_arr_file = "local_map_arr.txt"
        map_arr_file_path = os.path.join(save_maps, map_arr_file)
        with open(map_arr_file_path, 'w') as output:
            for i, grid in enumerate(msg.data, 1):
                output.write(str(grid))
                if i != len(msg.data):
                    output.write(",")
            output.write("\n")
            output.write("map_width: " + str(msg.info.width))
            output.write("\n")
            output.write("map_height: " + str(msg.info.height))
            output.write("\n")
            output.write("map_resolution: " + str(msg.info.resolution))
            output.write("\n")
            output.write("map_pose_x: " + str(msg.info.origin.position.x))      # do i need q?
            output.write("\n")
            output.write("map_pose_y: " + str(msg.info.origin.position.y))


        tracker = TB3Tracker()
        tracker.robot_i = robot_i
        tracker.start_area_i = start_area_i
        self.tracker_publisher.publish(tracker)

        # if (robot_i in start_area_i):
        #     self.get_logger().info("we made it back into the general vicinity")
        # if (robot_i == map_origin_i):
        #     self.get_logger().info("we made it to the exact original spot")

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
    node = RobotWorldNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
