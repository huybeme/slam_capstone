#!/usr.bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class tps_node(Node):
    def __init__(self):
        super().__init__("tps_node")

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser.reset_input_buffer()

        self.tps_publisher = self.create_publisher(
            Float32, "tps_node", 10
        )
        self.pub_timer = self.create_timer(0.1, self.callback_temp_publisher)
        
        self.get_logger().info("TPS10 node started")

    def callback_temp_publisher(self):

        self.ser.write(b"true")
        
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            try:
                line = float(line)
            except:
                return
            temp = Float32()
            temp.data = line
            self.tps_publisher.publish(temp)
            print(line)
        else:
            print("else statement")


def main(args=None):
    rclpy.init(args=args)
    node = tps_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()