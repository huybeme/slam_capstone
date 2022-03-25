import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos, pi

from tf_transformations import quaternion_from_euler


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__("cs_robot_state_publisher")

        self.joint_publisher = self.create_publisher(
            JointState, "join_states", 10
        )
        self.br = TransformBroadcaster(self, 10)

        degree = pi / 180.0
        rate = self.create_rate(30)
        self.timer = self.create_timer(1, self.transform_robot_state)

        # the robot state
        self.tilt = 0.
        self.tinc = degree
        self.swivel = 0.
        self.angle = 0.
        self.height = 0.
        self.hinc = 0.005

        self.odom_transform = TransformStamped()
        self.odom_transform.header.frame_id = "odom"
        self.odom_transform.child_frame_id = "axis"
        self.joint_state = JointState()

        self.get_logger().info("Robot state publisher has started")

    def transform_robot_state(self):
        pass



def main(args=None):
    rclpy.init(args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()