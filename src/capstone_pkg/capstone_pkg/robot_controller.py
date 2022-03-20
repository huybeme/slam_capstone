import rclpy
from rclpy import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.pose_subscriber = self.create_subscription(
            PoseStamped, 'pose', self.callback_pose, 10
        )

        self.transform_msg = Transform()
        self.q = Quaternion()
        self.p = Pose()
        self.br = TransformBroadcaster()

    def callback_pose(self):
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform



def main(args=None):

    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == "__main__":
    main()
