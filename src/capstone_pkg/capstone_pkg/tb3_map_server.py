import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import SetBool
from capstone_interfaces.msg import TB3Status
from capstone_interfaces.msg import TB3Tracker
from capstone_interfaces.action import Cartographer

import time
import subprocess
import os
import signal

# TODO: fix this so its not hard coded
cartographer_launch_file = "/home/hle/turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/launch/cartographer.launch.py"

class TB3MapServer(Node):
    def __init__(self):
        super().__init__("TB3_Action_Server_node")

        self.subscribe_tracker = self.create_subscription(
            TB3Tracker, "tb3_tracker", self.callback_tracker, 10
        )
        self.robot_i = None
        self.start_area_i = None

        self.launch_cartographer = None

        self.state = 0

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
        # self.get_logger().info("\n\n\n")

        # when the cartographer is closed, this node will still run at callback
        #   although since the cartographer is closed, robot world has no map to subscribe to
        # self.launch_cartographer.send_signal(signal.SIGINT)  # SIGINT is the same as ctrl + C
        # self.launch_cartographer.wait(timeout=10)
        pass
        
        if len(msg.start_area_i) > 0:
            self.robot_i = msg.robot_i
            self.start_area_i = msg.start_area_i
            # self.get_logger().info("tracker data recieved and tracking robot location")
        else:
            self.get_logger().info("tracker data not yet recieved")
            return


        if self.robot_i in self.start_area_i and self.state == 1:
            # os.system("ros2 run nav2_map_server map_saver_cli -f ~/filename")
            self.state = 2
            self.get_logger().info("Found original spot and saved map.")

        if self.state == 1:
            self.get_logger().info("still mapping")
            return

        if self.state == 2:
            self.get_logger().info("tracking complete, shutdowning cartographer now")
            # self.launch_cartographer.send_signal(signal.SIGINT)  # SIGINT is the same as ctrl + C
            # self.launch_cartographer.wait(timeout=10)           

        # cartographer_launch = subprocess.Popen(["ros2", "launch", cartographer_launch_file], text=True)
        # cartographer_launch.send_signal(signal.SIGINT)
        # cartographer_launch.wait(timeout=10)

    def map_action_callback(self, goal_handle):
        self.get_logger().info("got a client request...")
        
        fb_msg = Cartographer.Feedback()
        fb_msg.still_mapping = goal_handle.request.start_mapping
        fb_msg.test_mapping = True

        # for i in range(5):
        #     self.get_logger().info(str(i +1))
        #     time.sleep(1)

        goal_handle.publish_feedback(fb_msg)

        goal_handle.succeed()
        
        result = Cartographer.Result()
        result.found_origin = True


        self.get_logger().info(str(fb_msg))
        self.get_logger().info(str(goal_handle))
        self.get_logger().info(str(result))
        return result

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