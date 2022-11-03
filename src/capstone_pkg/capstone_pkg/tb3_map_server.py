import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import SetBool
from capstone_interfaces.msg import TB3Status
from capstone_interfaces.msg import TB3Tracker
from capstone_interfaces.action import Cartographer

import capstone_pkg.capstone_function as capstone_function

import time
import subprocess
import os
import signal

# TODO: fix this so its not hard coded
cartographer_launch_file = "/home/hle/turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/launch/cartographer.launch.py"
navigation_launch_file = "/home/hle/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/launch/navigation2.launch.py"

path = os.path.dirname(__file__)
root = os.path.abspath(os.path.join(path, "..", "..", ".."))
save_maps = os.path.join(root, "maps")

class TB3MapServer(Node):
    def __init__(self):
        super().__init__("TB3_Action_Server_node")

        self.subscribe_tracker = self.create_subscription(
            TB3Tracker, "tb3_tracker", self.callback_tracker, 10
        )
        self.robot_i = None
        self.start_area_i = None

        self.launch_cartographer = None
        self.launch_navigation = None

        self.state = 0
        self.left_initial_spot = False

        self.open_cartographer_check = self.create_service(
            SetBool, "map_server_check", self.callback_service_check
        )

        self.get_logger().info("TB3 map server node started")

    

    def callback_service_check(self, request, response):
        self.get_logger().info("waiting for a client...")
        req = request.data
        if req:
            self.robot_service_completed = req
            response.success = True
            response.message = "node ready, client request completed for robot world node"

            self.get_logger().info("launching cartographer now")
            self.launch_cartographer = subprocess.Popen(["ros2", "launch", cartographer_launch_file], text=True)
            return response
        else:
            response.success = False
            response.message = "function sent False as request data"
            return response

    def callback_tracker(self, msg):
        # self.get_logger().info(str(msg.robot_transform.transform))

        if self.launch_cartographer is None:
            self.get_logger().info("cartographer package not yet launched")
            return
        
        if len(msg.start_area_i) > 0:
            self.robot_i = msg.robot_i
            self.start_area_i = msg.start_area_i
            # self.get_logger().info("tracker data recieved and tracking robot location")
        else:
            self.get_logger().info("tracker data not yet recieved")
            return

        if self.robot_i in self.start_area_i:
            if self.state == 0:
                self.get_logger().info("still have not left initial area")
                return
            elif self.state == 1:

                # create unique map name and save
                timestamp = time.time_ns()
                map_name = "rviz2_map_" + str(timestamp)
                map_path = os.path.join(save_maps, map_name)
                map_command = "ros2 run nav2_map_server map_saver_cli -f " + map_path
                # os.system(map_command)

                # change state and shutdown cartographer
                self.get_logger().info("Found original spot and saved map. Cartographer closing now")
                # self.launch_cartographer.send_signal(signal.SIGINT)
                # self.launch_cartographer.wait(timeout=10)

                self.get_logger().info("launching navigation now")
                nav_args = "map:=" + map_path + ".yaml"
                self.launch_navigation = subprocess.Popen(["ros2", "launch", "/home/hle/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/launch/navigation2.launch.py",
                "map:=/home/hle/Desktop/compsci/ros/slam_capstone/maps/rviz2_map_1666906890637391581.yaml"], text=True)

                # self.launch_navigation.send_signal(signal.SIGINT)
                # self.launch_cartographer.wait(timeout=10)

                capstone_function.send_service_request(self, "robot_movement_state", "circle_around", 2)
                self.get_logger().info("closing navigation now")

                self.state = 2

        # might want this as elif and part of a state machine
        elif self.robot_i not in self.start_area_i and not self.left_initial_spot:
            self.left_initial_spot = True # at this point, it will always be true and will not re-enter this condiditon
            self.state = 1
            self.get_logger().info("left original spot")
        else:
            return
            self.get_logger().info("error, state is: " + str(self.state))

def main(args=None):
    rclpy.init(args=args)

    try:
        # subie = TB3_MapSubscriber()
        server = TB3MapServer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(server)
        # executor.add_node(subie)


        try:
            executor.spin()
        finally:
            executor.shutdown()
            # subie.destroy_node()
            server.destroy_node()
    finally:

        rclpy.shutdown()


if __name__ == "__main__":
    main()