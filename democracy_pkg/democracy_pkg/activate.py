import rclpy
from rclpy.node import Node
import argparse

from lx16a import *
from pylx16a.lx16a import LX16A
from math import sin, cos

from std_msgs.msg import String
from std_srvs.srv import SetBool
import random
from std_msgs.msg import String


class ActivateNode(Node):

    def __init__(self, voice_enabled=False):
        super().__init__('activate')
        self.get_logger().info("====== DEMOCRACY ACTIVATION PROCEDURE INITATED ======")

        self.voice_enabled = voice_enabled
        self.get_logger().info(f"  [MAIN]  Voice Service: {'ENABLED' if voice_enabled else 'DISABLED'}")

        # Publisher
        self.voice_request = self.create_publisher(String, 'voice_request', 10)

        # Service to toggle voice on/off
        self.voice_toggle_srv = self.create_service(
            SetBool,
            'toggle_voice',
            self.toggle_voice_callback
        )

        # Activation Line - On
        self.get_logger().info(" [MAIN]  Turning on.")
        self.voice_request.publish(quickLine("on", 1, 3))

        # Activation Line - Dianostics
        self.get_logger().info(" [MAIN]  RUNNING DIAGNOSTICS... ")
        self.voice_request.publish(quickLine("diagnostics", 1, 3))

        # LX16A.initialize('/dev/ttyUSB0')

        # try:
        #     servo1 = LX16A(1)
        #     servo1.set_id(1)
        # except (ServoTimeOutError, ServoChecksumError) as e:
        #     self.get_logger().error(f"Error initializing servo 1: {e}")
        #     return


    def toggle_voice_callback(self, request, response):
        self.voice_enabled = request.data
        state = "ENABLED" if self.voice_enabled else "DISABLED"
        self.get_logger().info(f" [MAIN]  Voice service {state}")
        response.success = True
        response.message = f" [MAIN]  Voice service {state}"
        return response


    def test_servo(self, servo):
        self.get_logger().info(f"Testing servo {servo.id}...")
        try:
            servo.move(90)
            self.get_logger().info(f"Servo {servo.id} moved to 90 degrees")
            rclpy.sleep(1)

            servo.move(0)
            self.get_logger().info(f"Servo {servo.id} moved to 0 degrees")
            rclpy.sleep(1)

        except (ServoTimeOutError, ServoChecksumError) as e:
            self.get_logger().error(f"Error testing servo {servo.id}: {e}")

    def initialization_movement(self):


        pass


def main(args=None):
    # Parse arguments before rclpy init
    parser = argparse.ArgumentParser(description='Activate Node')
    parser.add_argument(
        '--voice',
        type=str,
        choices=['on', 'off'],
        default='on',
        help='Enable or disable voice service on startup (on/off)'
    )

    # Parse only known args to avoid conflicts with ROS2 args
    parsed, _ = parser.parse_known_args()
    voice_enabled = parsed.voice == 'on'

    rclpy.init(args=args)

    node = ActivateNode(voice_enabled=voice_enabled)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()




def quickLine(name, min_var, max_var):
    variation = random.randint(min_var, max_var)
    filename = String()
    filename.data = f"{name}_{variation}_fr.mp3"
    return filename