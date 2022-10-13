from functools import partial
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import time

from capstone_interfaces.msg import TB3Status
from capstone_interfaces.msg import TB3Link

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

        self.robot_world_link = TB3Link()
        self.publish_robot_world_link = self.create_publisher(
            TB3Link, "tb3_link", 10
        )

        self.circle_service_check = self.create_service(
            SetBool, "circle_around_check", self.callback_service_check
        )

        self.state = 100

        self.get_logger().info("circling around node has begun")
    
    def callback_service_check(self, request, response):
        req = request.data
        if req:
            self.robot_service_completed = req
            response.success = True
            response.message = "node ready, client request completed for circle around node"
            return response
        else:
            response.success = False
            response.message = "function sent False as request data"
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

        if self.state == 2:  # not used
            self.stop_movement()
            self.publish_movement.publish(self.movement)
            self.movement.linear.x = 0.1
            time.sleep(2)
            self.stop_movement()

        if self.state == 99:
            self.stop_movement()

        if self.state == 100:
            # capstone_function.send_service_request(self, "robot_initialization")
            # self.send_request()
            self.robot_world_link.circle_around_link = True
            self.state = 1

        print("movement\n",
              "state: ", self.state,
              "\nx: ", float("{:.3f}".format(self.movement.linear.x)),
              "z: ", float("{:.3f}".format(self.movement.angular.z)))

        self.publish_movement.publish(self.movement)
        self.publish_robot_world_link.publish(self.robot_world_link)

    '''might not needed since we will put this in a module instead. will remove if I can figure out how
    to get rid of the publisher reigstered node name warning.'''
    def send_request(self):
        client = self.create_client(SetBool, 'robot_initialization')

        # send request is not asynchronous; node stuck in while loop with no other actions
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for robot world to recieve request")
        req = SetBool.Request()
        req.data = True

        future = client.call_async(req)

        future.add_done_callback(
            # response not used as variable, only for troubleshooting as messages
            partial(self.call_send_service, data=False)
        )
        self.get_logger().info("robot world node recieved request")

    def call_send_service(self, future, data):
        try:
            response = future.result()
            # "true" and "got your message"
            self.get_logger().info(str(response.success))
            self.get_logger().info(str(response.message))
        except:
            self.get_logger().info("service call failed")

    def move_along_wall(self, lidar):
        # may get stuck in concave that is just bigger than robot
        self.movement.linear.x = 0.1
        print("moving along wall")

        # hit a wall
        if lidar[1] < 0.38 and lidar[1] > 0.01:
            self.make_turn(1)

        # elif lidar[4] > 0.3 and lidar[4] < 0.5: # try to smooth out oscillation
        #     x = 2 * lidar[4] - 0.8
        #     self.movement.angular.z = x
        #     print(0, self.three_decimals(x), lidar[4])

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
            speed = 0.22 - ((-0.229 * lidar[1]) + 0.229)
            if speed > 0.22:
                self.movement.linear.x = 0.21
            else:
                # 0.21 - ((-0.229 * lidar[1]) + 0.229)
                self.movement.linear.x = speed
            if lidar[1] < 0.4:
                self.state = 100
                self.stop_movement()

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
