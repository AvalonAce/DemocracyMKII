import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

import board
import busio
import adafruit_lsm6ds.lsm6dsox
import adafruit_lis3mdl

import time

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        i2c = board.I2C()
        self.sox = LSM6DSOX(i2c)

        self.timer = self.create_timer(0.1, self.publish_imu)

    def publish_imu(self):
        accel = self.sox.acceleration
        gyro = self.sox.gyro

        self.get_logger().info(f"Accel: {accel}, Gyro: {gyro}")




def main(args=None):
    rclpy.init(args=args)

    node = ImuNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()