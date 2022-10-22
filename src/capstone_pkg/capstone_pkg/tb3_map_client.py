import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from capstone_interfaces.msg import TB3Status
from capstone_interfaces.action import Cartographer
import subprocess

"""
    client will send a server a request
"""

class TB3_MapClient(Node):
    def __init__(self):
        super().__init__("scratch_node")

        self.mapping_client = ActionClient(
            self, Cartographer, "tb3_mapping"
        )

        self.get_logger().info("scratch node started")
        self.send_goal(False)

    def send_goal(self, start_mapping):
        map_goal = Cartographer.Goal()
        map_goal.start_mapping = start_mapping
        self.get_logger().info("goal sent, waiting for server...")
        self.mapping_client.wait_for_server()

        self.send_goal_future = self.mapping_client.send_goal_async(
            map_goal, feedback_callback=self.callback_feedback
        )

        self.send_goal_future.add_done_callback(self.goal_msg_callback)

    def callback_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info("feedback: {0}".format(fb.still_mapping))

    def goal_msg_callback(self, future):
        goal_handle = future.result()
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        self.get_logger().info("here")
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.found_origin))
        rclpy.shutdown()

    def callback_status(self, msg):

        if self.state == 1:
            return

        # cartographer_launch = subprocess.Popen(["ros2", "launch", cartographer_launch_file], text=True)
        # cartographer_launch.send_signal(signal.SIGINT)
        # cartographer_launch.wait(timeout=10)
        

def main(args=None):

    rclpy.init(args=args)
    node = TB3_MapClient()
    rclpy.spin(node)
    # rclpy.shutdown()  # cannot have shutdowns at multiple spots, will get error somehow


if __name__ == "__main__":
    main()