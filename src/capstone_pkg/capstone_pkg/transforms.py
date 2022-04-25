import rclpy
from rclpy.node import Node
from tf2_ros import *
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.qos import qos_profile_sensor_data
import laser_geometry.laser_geometry as lg
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
import tf_transformations

class TransformerNode(Node):
    def __init__(self):
        super().__init__("transformer_node")
        
        self.timer = self.create_timer(0.05, self.on_timer)

        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.callback_odom, 10
        )

        self.br = TransformBroadcaster(self)
        self.bscan = TransformStamped()
        self.bscan.header.frame_id = "base_link"
        self.bscan.child_frame_id = "base_scan"

    def callback_odom(self, msg):
        
        self.bscan.transform.translation.x = msg.pose.pose.position.x
        
        print(msg.pose.pose.position)

    def on_timer(self):
        self.bscan.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.bscan)
        


def main(args=None):
    rclpy.init(args=args)
    node = TransformerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()