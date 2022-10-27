from functools import partial
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import time
import signal

from capstone_interfaces.msg import TB3Status
from capstone_interfaces.msg import TB3Link
from capstone_interfaces.srv import State

import capstone_pkg.capstone_function as capstone_function


class circle_around_node(Node):
    def __init__(self):
        super().__init__("circle_around")

        self.tb_status = TB3Status()
        self.get_lidar_val = self.create_subscription(
            TB3Status, "tb3_status", self.callback_lidar_and_move, 10)

        self.movement = Twist()
        self.publish_movement = self.create_publisher(
            Twist, "cmd_vel", 10
        )

        self.circle_service_check = self.create_service(
            State, "robot_movement_state", self.callback_movement_state
        )

        self.state = 0

        
        signal.signal(signal.SIGINT, self.user_stop_movement)
        self.get_logger().info("circling around node has begun")
    
    def callback_movement_state(self, request, response):
        req = request.state
        if type(req) is int:
            self.state = req
            response.success = True
            return response
        else:
            response.success = False
            return response

    def callback_lidar_and_move(self, msg):
        # 1m = 39.4 in      12 in = 0.3m    1in = 0.025m    0.1m = 3.9in
        # max linear = 0.22   max angular = 2.84
        # positive z = right
        # [2] 0.5 = [4] 0.3 apprx
        if self.state == 0:
            self.setup_initial_state(msg.lidar_data)

        if self.state == 1:
            self.move_along_wall(msg.lidar_data)

        if self.state == 99:
            self.stop_movement()

        if self.state == 100:
            # does not print until request is completed
            print(capstone_function.send_service_request(self, "robot_world_check", "robot_world"))
            self.state = 1

        print("movement\n",
              "state: ", self.state,
              "\nx: ", float("{:.3f}".format(self.movement.linear.x)),
              "z: ", float("{:.3f}".format(self.movement.angular.z)))

        self.publish_movement.publish(self.movement)

    def move_along_wall(self, lidar):
        # may get stuck in concave that is just bigger than robot
        self.movement.linear.x = 0.1
        print("moving along wall")

        # hit a wall
        if lidar[1] < 0.38 and lidar[1] > 0.01:
            self.make_turn(1)

        # align to wall
        elif lidar[4] > 0.003 and lidar[4] <= 1.0:
            x = (lidar[2] - 0.4) * 1.5
            x = self.cube_root(x)
            self.movement.angular.z = -x  # -0.357 * lidar[4] + 0.15
            print(1, self.three_decimals(x), lidar[4])

        else:
            self.movement.angular.z = 0.0

    def setup_initial_state(self, lidar):

        if lidar[1] < 0.4 and lidar[1] > 0.01:    # too close to wall
            self.movement.linear.x = -0.05
            if lidar[1] > 0.39:
                self.state = 100
                self.stop_movement()

        elif lidar[1] > 0.39:
            if lidar[1] < 0.4:
                self.state = 100
                self.stop_movement()
            speed = 0.22 - ((-0.229 * lidar[1]) + 0.229)
            if speed > 0.22:
                self.movement.linear.x = 0.21
            else:
                # 0.21 - ((-0.229 * lidar[1]) + 0.229)
                self.movement.linear.x = speed

    def make_turn(self, direction):
        # direction -1 == left  1 == right
        print("turning")
        self.movement.linear.x = 0.0
        self.movement.angular.z = 0.9 * direction
        self.publish_movement.publish(self.movement)
        time.sleep(0.5)
        self.stop_movement()

    def stop_movement(self):
        self.movement.angular.z = 0.0
        self.movement.linear.x = 0.0
        self.publish_movement.publish(self.movement)
        # should I wait a few seconds here?

    def user_stop_movement(self, signal_received, frame):
        self.stop_movement()
        self.state = -99
        self.get_logger().info("user stoped circle around movement")
        exit(0)

    def cube_root(self, x):
        ''' prevent real number output done by python, this helps prevent output of real number
            https://stackoverflow.com/questions/42189413/wrong-value-for-cube-root-in-python'''
        if x >= 0:
            return x ** (1/3)
        else:
            return -(-x) ** (1/3)

    def three_decimals(self, x):
        return '{0:.3g}'.format(x)


def main(args=None):
    rclpy.init(args=args)
    node = circle_around_node()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
