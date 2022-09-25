# getting temperature from TMP117 through I2C and RPi
# using libraries from adafruit - https://github.com/adafruit/Adafruit_CircuitPython_TMP117
# and embedded it into a ros2 framework
#!/usr.bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import board
import adafruit_tmp117


class tmp_node(Node):
    def __init__(self):
        super().__init__("tmp_node")

        self.i2c = board.I2C()
        self.tmp_sensor = adafruit_tmp117.TMP117(self.i2c)

        self.tmp_publisher = self.create_publisher(
            Float32, "tmp_node", 10
        )
        self.pub_timer = self.create_timer(0.1, self.callback_temp_publisher)

        self.get_logger().info("tmp117 node started")


    def callback_temp_publisher(self):

        # temperature sensor is not calibrated - not necessary
        temp_c = self.tmp_sensor.temperature
        temp_f = temp_c * 1.8 + 32

        temp = Float32()
        temp.data = temp_f
        self.tmp_publisher.publish(temp)



def main(args=None):
    rclpy.init(args=args)
    node = tmp_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
