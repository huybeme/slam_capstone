import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, TypeException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import time


class OccupancyNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid")
        self.resolution = 0.05

        self.local_dimension = 10.0
        self.local_size_x = int(self.local_dimension / self.resolution)
        self.local_size_y = int(self.local_dimension / self.resolution)
        self.local_origin = -1 * self.local_dimension / 2               # x and y
        self.local_size = int(self.local_dimension / self.resolution)

        self.local_grid = OccupancyGrid()
        self.local_grid.header.frame_id = "odom"
        self.local_grid.info.resolution = self.resolution
        self.local_grid.info.width = self.local_size_x
        self.local_grid.info.height = self.local_size_y
        self.local_grid.info.origin.position.x = self.local_origin
        self.local_grid.info.origin.position.y = self.local_origin
        self.local_grid.info.origin.orientation.w = 1.0
        self.local_grid.data = [-1] * int(self.local_size_x * self.local_size_y)

        self.local_theta = 0
        self.local_current_x = 0
        self.local_current_y = 0

        # variable last 0.0/theta_offset
        local_q = quaternion_from_euler(0.0, 0.0, 0.0)
        self.laser_transformer = TransformStamped()
        self.laser_transformer.header.frame_id = "base_link"
        self.laser_transformer.child_frame_id = "base_scan"
        self.laser_transformer.transform.translation.x = 0.0    # laseroffsetx
        self.laser_transformer.transform.translation.y = 0.0    #
        self.laser_transformer.transform.rotation.x = local_q[0]
        self.laser_transformer.transform.rotation.y = local_q[1]
        self.laser_transformer.transform.rotation.z = local_q[2]
        self.laser_transformer.transform.rotation.w = local_q[3]

        self.global_dimension = 20
        self.global_size_x = int(self.global_dimension / self.resolution)
        self.global_size_y = int(self.global_dimension / self.resolution)
        self.global_origin = -1 * self.global_dimension / 2
        self.global_size = int(self.global_dimension / self.resolution)

        self.global_grid = OccupancyGrid()
        self.global_grid.header.frame_id = "global_world"
        self.global_grid.info.resolution = self.resolution
        self.global_grid.info.height = self.global_size_y
        self.global_grid.info.width = self.global_size_x
        self.global_grid.info.origin.position.x = self.global_origin
        self.global_grid.info.origin.position.y = self.global_origin
        self.global_grid.info.origin.orientation.w = 1.0
        self.global_grid.data = [-1] * int(self.global_size_x * self.global_size_y)
        self.global_grid_publisher = self.create_publisher(
            OccupancyGrid, "global_map", 10
        )

        self.laser_subscriber = self.create_subscription(
            LaserScan, "scan", self.callback_laser_data, qos_profile_sensor_data
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.callback_odometry, 10
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid, "map", 10
        )
        self.map_publisher_timer = self.create_timer(0.5, self.publish_map)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.laser_ready = False

        self.get_logger().info("Occupancy Node has started")

    def publish_map(self):
        self.map_publisher.publish(self.local_grid)
        # self.global_grid_publisher.publish(self.global_grid)

    def callback_odometry(self, msg):
        self.local_current_x = msg.pose.pose.position.x
        self.local_current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        orientation = (orientation.x, orientation.y,
                       orientation.z, orientation.w)
        self.local_theta = euler_from_quaternion(orientation)[2]

    def callback_laser_data(self, msg):

        # get the sensor angle, happens at every degree
        robot_angles = []
        limit_robot_max_range = 1.0
        for i in range(len(msg.ranges)):
            robot_angles.append(msg.angle_min + i * msg.angle_increment)

        # robot starting point
        grid_cord_xy0 = self.world_to_grid(self.local_current_x, self.local_current_y)
        # print(grid_cord_xy0)

        # fill in grid data at each angle
        for i, ranges in enumerate(msg.ranges):
            limited_range = 0.0
            if msg.ranges[i] > limit_robot_max_range:
                limited_range = limit_robot_max_range
            else:
                limited_range = msg.ranges[i]
            # get coords from end point and convert to grid coords
            world_x = limited_range * math.cos(robot_angles[i] + self.local_theta) + self.local_current_x
            world_y = limited_range * math.sin(robot_angles[i] + self.local_theta) + self.local_current_y
            grid_cord_xy1 = self.world_to_grid(world_x, world_y)

            # print(world_x, world_y)
            # print(grid_cord_xy0, grid_cord_xy1)

            # returns an array of points from center to end
            bresenham_path = self.bresenham(grid_cord_xy0[0], grid_cord_xy0[1], grid_cord_xy1[0], grid_cord_xy1[1])
            
            # make everything from start to end free
            for x, y in bresenham_path:
                index = y * self.local_size_x + x
                self.local_grid.data[index] = 0

            # anything within range, occupy
            index = grid_cord_xy1[1] * self.local_size_x + grid_cord_xy1[0]
            if ranges < limit_robot_max_range:
                self.local_grid.data[index] = 100


    def bresenham(self, x0, y0, x1, y1):
        # algoirth source: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        
        if abs(y1 - y0) < abs(x1 - x0):
            if x0 > x1:
                return self.bresenham_low(x1, y1, x0, y0)
            else:
                return self.bresenham_low(x0, y0, x1, y1)
        else:
            if y0 > y1:
                return self.bresenham_high(x1, y1, x0, y0)
            else:
                return self.bresenham_high(x0, y0, x1, y1)

    def bresenham_low(self, x0, y0, x1, y1):
        dx = x1 - x0
        dy = y1 - y0
        yi = 1
        if dy < 0:
            yi = -1
            dy = -dy
        
        D = (2 * dy) - dx
        y = y0

        path = []
        for x in range(int(x0), int(x1)):
            coord = (x, y)
            path.append(coord)
            if D > 0:
                y += yi
                D = D + (2 * (dy - dx))
            else:
                D = D + 2 * dy
        return path

    def bresenham_high(self, x0, y0, x1, y1):
        dx = x1 - x0
        dy = y1 - y0
        xi = 1
        if dx < 0:
            xi = -1
            dx = -dx
        D = (2 * dx) - dy
        x = x0

        path = []

        for y in range(int(y0), int(y1)):
            coord = (x, y)
            path.append(coord)
            if D > 0:
                x = x + xi
                D = D + (2 * (dx - dy))
            else:
                D = D + 2 * dx
        return path


    def swap(self, a, b):
        return [b, a]

    def grid_to_world(self, grid_x, grid_y):
        pass

    def world_to_grid(self, x, y):
        if (x < self.local_origin or x >= self.resolution * self.local_size_x) or \
            (y < self.local_origin or y >= self.local_size_y * self.local_size_y):
            # print(x, y, self.local_origin)
            return [1,1]

        gx = int((x - self.local_origin) / self.resolution)
        gy = int((y - self.local_origin) / self.resolution)

        # print(x, y, gx, gy)
        return (gx, gy)


def main(args=None):

    rclpy.init(args=args)
    node = OccupancyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
