# getting temperature from TMP117 through I2C and RPi4
# accessing proper addresses and sensor value conversions taken from
# https://learn.sparkfun.com/tutorials/python-programming-tutorial-getting-started-with-the-raspberry-pi
# in order to embedded into a ROS2 framework

#!/usr.bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import smbus


class tmp_node(Node):
    def __init__(self):
        super().__init__("tmp_node")

        self.i2c_ch = 1
        self.i2c_addr = 0x48

        self.reg_temp = 0x00
        self.reg_config = 0x01

        self.bus = smbus.SMBus(self.i2c_ch)

        self.val = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_config, 2)
        print("old config: ", self.val)
        self.val[1] = self.val[1] & 0b00111111
        self.val[1] = self.val[1] | (0b11 << 6)

        self.bus.write_i2c_block_data(self.i2c_addr, self.reg_config, self.val)

        self.val = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_config, 2)
        print("new config: ", self.val)


        self.tmp_publisher = self.create_publisher(
            Float32, "tmp_node", 10
        )
        self.pub_timer = self.create_timer(0.1, self.callback_temp_publisher)
        
        self.get_logger().info("tmp117 node started")

    def twos_comp(val, bits, self):
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val

    def read_temp(self):

        val = self.bus.read_i2c_block_data(self.i2c_addr, self.reg_temp, 2)
        temp_c = (val[0] << 4) | (val[1] >> 4)
        temp_c = self.twos_comp(temp_c, 12)
        temp_c *= 0.0625
        
        return temp_c


    def callback_temp_publisher(self):

        temp_c = self.read_temp()
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