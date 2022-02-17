import os
import rclpy
import time

# Twist type contains two Vector3 types (linear and angular)
# Vector3 type contains three floats (x, y, z)
# together these will allow the robot to move in any direction
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile

# max speeds
MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84

# get the model from source
TB3_MODEL = os.environ['TURTLEBOT3_MODEL']

def main():

    rclpy.init()

    qos = QoSProfile(depth=10)  # buffer space?
    node = rclpy.create_node('basic_movement')
    # publish to cmd_vel topic, this topic will accept Twist values for movement
    publish_movement = node.create_publisher(Twist, 'cmd_vel', qos)

    twist = Twist()

    for i in range(3):

        # ground units will only need linear.x for forwards and backwards
        # and angular.z for left and right

        twist.linear.x = 0.1
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        publish_movement.publish(twist)

        time.sleep(1)

    # turn off movement
    twist.linear.x = 0.0
    publish_movement.publish(twist)