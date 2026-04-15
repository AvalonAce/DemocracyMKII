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
import time
import threading


class ActivateNode(Node):

    def __init__(self, voice_enabled=False):
        super().__init__('activate')
        self.get_logger().info("====== DEMOCRACY ACTIVATION PROCEDURE INITATED ======")

        self.voice_enabled = voice_enabled
        self.on_watch = False
        
        self.get_logger().info(f"  [MAIN]  Voice Service: {'ENABLED' if voice_enabled else 'DISABLED'}")

        # Service to toggle voice on/off
        self.voice_toggle_srv = self.create_service(
            SetBool,
            'toggle_voice',
            self.toggle_voice_callback
        )

        self.idle_toggle_srv = self.create_service(
        SetBool,
        'set_idle',
        self.set_idle_callback
        )
        

        # Publisher
        self.voice_request = self.create_publisher(String, 'voice_request', 10)
        self.servo_preset_command = self.create_publisher(String, 'movement_preset', 10)

        # Activation Sequence
        self.bypass_activation = True
        if not self.bypass_activation:
            self.create_timer(3.0, self._run_activation_sequence)  # fires once after 2s
            self._activation_done = False
        else: 
            self.get_logger().warn("Bypassing activation sequence.")
            time.sleep(1)
            self.voice_request.publish(quickLine("copy", 1, 2))
            self.voice_enabled = False


        self._cmd_thread = threading.Thread(target=self.command, daemon=True)
        self._cmd_thread.start()
        
        
    def _run_activation_sequence(self):
        if self._activation_done:
            return
        self._activation_done = True  # prevent re-firing

        # Activation Line - On
        self.get_logger().info(" [MAIN]  Turning on.")
        self.voice_request.publish(quickLine("on", 1, 3))

        # Activation Line - Dianostics
        self.get_logger().info(" [MAIN]  RUNNING DIAGNOSTICS... ")
        self.voice_request.publish(quickLine("diagnostics", 1, 3))
        time.sleep(3)

        # Run Init movment of servos
        self.servo_preset_command.publish(String(data="initialize"))
        time.sleep(11)

        # Activation Line - Good to Go (assumed)
        self.get_logger().info(" [MAIN]  All systems functional")
        self.voice_request.publish(quickLine("nomi", 1, 2))


    def command(self):
        commands = {
            "help"    : self.cmd_help,
            "status"  : self.cmd_status,
            "stop"    : self.cmd_stop,
            "walk"    : self.cmd_walk,
            "home"    : self.cmd_home,
            "dance"   : self.cmd_dance,
            "run"     : self.cmd_run,
            "idle"    : self.cmd_idle,
        }

        self.get_logger().info("Command interface ready. Type 'help' for available commands.")

        while rclpy.ok():
            try:
                raw = input(">> ").strip()
            except EOFError:
                break

            if not raw:
                continue

            parts = raw.split()
            name, args = parts[0].lower(), parts[1:]

            if name in commands:
                self.on_watch = False  # Cancel idle loop on any new command
                commands[name](*args)
            else:
                print(f"Unknown command: '{name}'. Type 'help' for available commands.")

    def cmd_help(self):
        print("\nAvailable commands:")
        print("  help              Show this message")
        print("  status            Print current robot state")
        print("  stop              Send a stop/zero-velocity command")
        print("  move <x> <y>      Move to target position (placeholder)\n")

    def cmd_status(self):
        # TODO: replace with real state readout from your node's fields
        self.get_logger().info("Status requested.")
        print("  [status] Robot is nominal (placeholder)")

    def cmd_stop(self):
        self.get_logger().info("Stop command issued.")
        self.voice_request.publish(quickLine("capitain", 1, 3))
        self.servo_preset_command.publish(String(data="stop"))
        print("  [stop] Halting all motion.")

    def cmd_walk(self, *args):
        if len(args) < 1:
            print("  Usage: walk <cycles>")
            return
        try:
            cycles = int(args[0])
        except ValueError:
            print("  Error: cycles must be a number.")
            return

        self.get_logger().info(f"Moving in a straight line to target.")
        self.voice_request.publish(quickLine("capitain", 1, 3))
        self.servo_preset_command.publish(String(data=f"walk {cycles}"))

    def cmd_run(self, *args):
        if len(args) < 1:
            print("  Usage: run <cycles>")
            return
        try:
            cycles = int(args[0])
        except ValueError:
            print("  Error: cycles must be a number.")
            return

        self.get_logger().info(f"Moving in a straight line to target.")
        self.voice_request.publish(quickLine("dust", 1, 3))
        self.servo_preset_command.publish(String(data=f"run {cycles}"))

    def cmd_home(self):
        self.get_logger().info("Home command issued. Returning to home position.")
        self.servo_preset_command.publish(String(data="home"))

    def cmd_dance(self):
        self.get_logger().info("Dance command issued. Initiating dance sequence.")
        self.voice_request.publish(quickLine("bust", 1, 4))
        self.servo_preset_command.publish(String(data="dance"))

    def cmd_idle(self):
        self.get_logger().info("Idle command issued. Entering idle walk.")
        self.voice_request.publish(quickLine("copy", 1, 2))
        self._start_idle()



    def toggle_voice_callback(self, request, response):
        self.voice_enabled = request.data
        state = "ENABLED" if self.voice_enabled else "DISABLED"
        self.get_logger().info(f" [MAIN]  Voice service {state}")
        response.success = True
        response.message = f" [MAIN]  Voice service {state}"
        return response
    
    def set_idle_callback(self, request, response):
        if request.data:
            self._start_idle()
            state = "ACTIVE"
        else:
            self.on_watch = False
            state = "STOPPED"

        self.get_logger().info(f" [MAIN]  Idle watch {state}")
        response.success = True
        response.message = f"Idle watch {state}"
        return response

    def _start_idle(self):
        if self.on_watch:
            self.get_logger().info("Idle loop already running.")
            return

        self.on_watch = True
        self.servo_preset_command.publish(String(data="idle"))

        def _idle_loop():
            while self.on_watch:
                time.sleep(10)

                random_num = random.randint(1, 5)
                if random_num == 1:
                    self.voice_request.publish(quickLine("yawn", 1, 2))
                if random_num == 2:
                    self.voice_request.publish(quickLine("standing", 1, 2))
                if random_num == 3:
                    self.voice_request.publish(quickLine("lovely", 1, 2))
                if random_num == 4:
                    self.voice_request.publish(quickLine("sleeps", 1, 2))
                if random_num == 5:
                    self.voice_request.publish(quickLine("landed", 1, 2))




            self.get_logger().info("Idle loop exited.")

        threading.Thread(target=_idle_loop, daemon=True).start()

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