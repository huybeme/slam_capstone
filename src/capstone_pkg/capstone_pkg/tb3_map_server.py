import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from capstone_interfaces.msg import TB3Status
from capstone_interfaces.action import Cartographer

import time

cartographer_launch_file = "/home/hle/turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/launch/cartographer.launch.py"

class TB3_MapSubscriber(Node):
    def __init__(self):
        super().__init__("TB3_Map_Server_Subscriber")

        self.subscribe_status = self.create_subscription(
            TB3Status, "tb3_status", self.callback_status, 10
        )

        self.test_y = None

        self.get_logger().info("TB3 action subscriber node started")

    def callback_status(self, msg):
        
        self.test_y = msg.robot_pos_y
        self.get_logger().info("{:.2e}".format(self.test_y))


class TB3MapServer(Node):
    def __init__(self):
        super().__init__("TB3_Action_Server_node")

        self.mapping_server = ActionServer(
            self, Cartographer, "tb3_mapping", self.map_action_callback
        )

        self.get_logger().info("TB3 action server node started")

    def map_action_callback(self, goal_handle):
        self.get_logger().info("got a client request...")
        
        fb_msg = Cartographer.Feedback()
        fb_msg.still_mapping = goal_handle.request.start_mapping

        for i in range(5):
            self.get_logger().info(str(i))
            time.sleep(1)

            goal_handle.publish_feedback(fb_msg)

        goal_handle.succeed()
        
        result = Cartographer.Result()
        result.found_origin = False
        return result

def main(args=None):
    rclpy.init(args=args)

    try:
        subie = TB3_MapSubscriber()
        server = TB3MapServer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(server)
        executor.add_node(subie)


        try:
            executor.spin()
        finally:
            executor.shutdown()
            subie.destroy_node()
            server.destroy_node()
    finally:

        rclpy.shutdown()


if __name__ == "__main__":
    main()