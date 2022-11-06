from functools import partial
import rclpy
import os
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from std_srvs.srv import SetBool
import numpy as np

from capstone_interfaces.msg import TB3Tracker
from capstone_interfaces.srv import Temperature

AREA = [
    [-1, 2], [0, 2], [1, 2],
    [-2, 1], [-1, 1], [0, 1], [1, 1], [2, 1],
    [-2, 0], [-1, 0], [0, 0], [1, 0], [2, 0],
    [-2, -1], [-1, -1], [0, -1], [1, -1], [2, -1],
    [-1, -2], [0, -2], [1, -2]
]

path = os.path.dirname(__file__)
root = os.path.abspath(os.path.join(path, "..", "..", ".."))
map_path = os.path.join(root, "maps")


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
        self.robot_transform = TransformStamped()

        # will get a bool from circle_around node
        self.robot_world_service_check = self.create_service(
            SetBool, "robot_world_check", self.callback_service_check
        )
        self.robot_service_completed = False

        self.start_found = False
        self.start_xy_pose = None
        self.start_xy_grid = None
        self.start_i = None

        self.tracker_publisher = self.create_publisher(
            TB3Tracker, "tb3_tracker", 10
        )

        self.find_hotspot_service = self.create_service(
            SetBool, "robot_world_temp", self.callback_hotspot_service
        )

        self.find_hot_spot = False

        self.current_tempF = 0.0
        self.max_tempF = None
        self.temp_logger = []       # might need to handle how large this gets

        self.state = 0

        self.get_logger().info("robot world started")

    def callback_hotspot_service(self, request, response):
        req = request.data
        self.get_logger().info("hot spot")
        if req:
            self.find_hot_spot = req
            response.success = True
            response.message = "robot_world ready to send robot to hotspot"
            return response
        else:
            response.success = False
            response.message = "robot_world did not send robot to hotspot"
            return response

    def callback_service_check(self, request, response):
        # wait for robot to finish initialzation and then trigger to take robot cords
        req = request.data
        if req:
            self.robot_service_completed = req
            response.success = True
            response.message = "robot localization initiated for robot_world"
            return response
        else:
            response.success = False
            response.message = "robot localization not initiated for robot_world"
            return response

    # get transforms - these frames will provide robot poses relative to map frame
    def on_timer(self):

        # try to find map > odom transform
        try:
            from_frame = self.target_frame
            to_frame = 'map'

            now = self.get_clock().now().to_msg()
            # go back in time n sec to get last frame since current frame "will not exist"
            now.nanosec = now.nanosec - 200000000
            trans = self.buffer.lookup_transform(
                to_frame,
                from_frame,
                now
            )

            self.vector = trans.transform.translation
            self.q = trans.transform.rotation

            self.robot_transform = trans

        except:
            return

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
        grid_file_path = os.path.join(map_path, grid_file)

        temp_file = "temp_grid.txt"
        temp_file_path = os.path.join(map_path, temp_file)
        temp_file_f = open(temp_file_path, "w")


        # this text file can be used to build OccupancyGrid data. can be used to publish latest map data
        # not really used at this point
        grid_arr_file = "temp_arr.txt"
        grid_arr_file_path = os.path.join(map_path, grid_arr_file)
        grid_arr_f = open(grid_arr_file_path, "w")

        with open(grid_file_path, 'w') as output:
            for i, grid in enumerate(msg.data, 1):

                if i == robot_i:
                    temp_file_f.write("[" + "{:.2f}".format(self.current_tempF) + "]")
                else:
                    temp_file_f.write("[xx.xx]")

                grid_arr_f.write(str(grid))
                if i != len(msg.data):
                    grid_arr_f.write(",")

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
                    temp_file_f.write("\n")

            grid_arr_f.write("\n")
            grid_arr_f.write("map_width: " + str(msg.info.width))
            grid_arr_f.write("\n")
            grid_arr_f.write("map_height: " + str(msg.info.height))
            grid_arr_f.write("\n")
            grid_arr_f.write("map_resolution: " + str(msg.info.resolution))
            grid_arr_f.write("\n")
            # do i need q?
            grid_arr_f.write("map_pose_x: " + str(msg.info.origin.position.x))
            grid_arr_f.write("\n")
            grid_arr_f.write("map_pose_y: " + str(msg.info.origin.position.y))

        temp_file_f.close()
        grid_arr_f.close()

        tracker = TB3Tracker()
        tracker.robot_i = robot_i
        tracker.start_area_i = start_area_i
        tracker.robot_transform = self.robot_transform
        self.tracker_publisher.publish(tracker)

        self.request_temp()     # should we get this returned instead?
        if self.max_tempF is None or self.max_tempF < self.current_tempF:
            self.get_logger().info("current max temp: " + str(self.max_tempF))
            self.get_logger().info("got new max temp: " + str(self.current_tempF))
            self.max_tempF = self.current_tempF
            self.temp_logger.append(
                (self.max_tempF, self.vector.x, self.vector.y))
        

        # average sensor when running the robot temp for a while is ~80 F
        if self.find_hot_spot:
            hotspot = 0.0
            hotspot_i = -99
            for i, temp in enumerate(self.temp_logger):
                if temp[0] > hotspot:
                    hotspot = temp[0]
                    hotspot_i = i
            
            self.get_logger().info(str(hotspot))
            self.get_logger().info(str(hotspot_i))
            self.get_logger().info(str(self.temp_logger[hotspot_i]))
            self.find_hot_spot = False


    def to_index(self, x, y, width):
        return (y * width + x)

    def world_to_grid(self, x, y, origin_x, origin_y, resolution, size_x, size_y):
        # if outside of grid
        if (x < origin_x or x >= resolution * size_x) or \
                (y < origin_y or y >= size_y * size_y):
            return [1, 1]

        gx = int((x - origin_x) / resolution)
        gy = int((y - origin_y) / resolution)

        return [gx, gy]

    def request_temp(self):
        client = self.create_client(Temperature, "temperature_service")

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for temperature server to respond")

        req = Temperature.Request()

        future = client.call_async(req)

        future.add_done_callback(
            partial(self.call_send_service)
        )

    def call_send_service(self, future):
        try:
            response = future.result()
            self.current_tempF = response.temperature
        except:
            return


def main(args=None):
    rclpy.init(args=args)
    node = RobotWorldNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
