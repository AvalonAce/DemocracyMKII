import rclpy
from rclpy.node import Node

from pylx16a.lx16a import LX16A, ServoTimeoutError
from math import sin, pi
import numpy as np
import time

import sys
import tty
import termios

def find_resting_position(servo: LX16A, step: float = 1.0, start_angle: float = 120.0) -> float:
    """
    Interactively find the best resting angle for a servo.
    Press A (backward), D (forward), Enter to confirm, Ctrl+C to cancel.

    Args:
        servo:       The LX16A servo instance to tune.
        step:        Degrees to move per keypress.
        start_angle: Starting angle for tuning (defaults to midpoint).

    Returns:
        The confirmed resting angle as a float.
    """
    angle = start_angle
    servo.move(angle)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    print(f"\n── Resting position tuner (servo #{servo.get_id()}) ──")
    print(f"  A = backward ({step}°)  |  D = forward ({step}°)")
    print(f"  Enter = confirm  |  Ctrl+C = cancel")
    print(f"  Current angle: {angle:.1f}°", end="\r")

    try:
        tty.setraw(fd)
        while True:
            key = sys.stdin.read(1).lower()

            if key == 'a':
                angle = max(0.0, angle - step)
            elif key == 'd':
                angle = min(240.0, angle + step)
            elif key in ('\r', '\n'):
                break
            elif key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt

            servo.move(angle)
            # \r rewrites the same line cleanly
            sys.stdout.write(f"  Current angle: {angle:.1f}°  \r")
            sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n  Cancelled. Returning to {start_angle:.1f}°")
        servo.move(start_angle)
        return start_angle

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    print(f"\n  ✓ Confirmed resting angle: {angle:.1f}°\n")
    return angle


def interpolate_leg(target_angle: float, steps: int = 20, curve: str = 'cubic') -> list[float]:
    """
    Generate an array of intermediate angles from 0 to target_angle.

    Args:
        target_angle: Final destination angle in degrees.
        steps:        Number of intermediate positions (more = smoother).
        curve:        Motion profile. One of:
                        'linear'  — constant rate
                        'sine'    — sinusoidal ease (smooth wave)
                        'cubic'   — cubic ease-in/out (slow start + slow stop)
                        'quad'    — quadratic ease-in (accelerates into position)
                        'exp'     — exponential (sharp at start, plateaus at end)

    Returns:
        List of angle values (floats) from near-0 to target_angle, length = steps.
    """
    t = np.linspace(0.0, 1.0, steps)

    if curve == 'linear':
        weights = t

    elif curve == 'sine':
        # Full sine arch: starts and ends near zero, peaks at midpoint
        weights = np.sin(t * pi)
        # Re-map so the integral reaches 1.0 (i.e. cumulative sum → target)
        weights = np.cumsum(weights)
        weights /= weights[-1]

    elif curve == 'cubic':
        # Smoothstep: 3t² - 2t³
        weights = 3 * t**2 - 2 * t**3

    elif curve == 'quad':
        # Quadratic ease-in: accelerates from zero
        weights = t**2

    elif curve == 'exp':
        # Exponential ease: very slow start then rapid climb
        weights = (np.exp(4 * t) - 1) / (np.exp(4) - 1)

    else:
        raise ValueError(f"Unknown curve '{curve}'. "
                         f"Choose from: 'linear', 'sine', 'cubic', 'quad', 'exp'.")

    return (weights * target_angle).tolist()


class ServoNode(Node):

    def __init__(self):
        super().__init__('servo_node')

        self.get_logger().info("Initializing LX16A servo node...")

        LX16A.initialize("/dev/ttyUSB0")

        try:
            self.torso  = LX16A(1)
            self.lKnee  = LX16A(2)
            self.rKnee  = LX16A(3)
            self.lCalf  = LX16A(4)
            self.rCalf  = LX16A(5)
            self.lFoot  = LX16A(6)
            self.rFoot  = LX16A(7)

            self.torso.set_angle_limits(0, 240)

        except ServoTimeoutError as e:
            self.get_logger().error(f"Servo {e.id_} not responding. Shutting down.")
            raise SystemExit

        self.t = 0.0
        # self.timer = self.create_timer(0.05, self.test_servo)
        # In your ServoNode.__init__, after the servos are initialized:
        #rest_angle = find_resting_position(self.rFoot, step=2.0)
        # self.get_logger().info(f"lKnee resting angle set to {rest_angle:.1f}°")
        self.return_to_rest()
        # self.test_legs()
        # self.rFoot.move(120)


    def test_servo(self):
        try:
            angle = sin(self.t) * 60

            # Example: move lKnee through a cubic ease curve to the target angle
            curve_steps = interpolate_leg(angle, steps=10, curve='cubic')
            for step_angle in curve_steps:
                self.lKnee.move(step_angle)

            self.get_logger().info(f"Servo angle: {angle:.2f}")
            self.t += 0.1

        except ServoTimeoutError as e:
            self.get_logger().error(f"Lost communication with servo {e.id_}")

    def return_to_rest(self, steps: int = 70, step_delay: float = 0.02, curve: str = 'cubic'):
        """
        Smoothly move all 7 servos to their resting positions simultaneously.

        Args:
            steps:      Number of interpolation steps (more = slower/smoother).
            step_delay: Seconds between each step.
            curve:      Motion profile passed to interpolate_leg().
        """
        REST_POSITIONS = {
            'torso':  110.0,
            'lKnee':  100.0,
            'rKnee':  130.0,
            'lCalf':  124.0,
            'rCalf':  100.0,
            'lFoot':  106.0,
            'rFoot':  106.0,
        }

        servos = {
            'torso':  self.torso,
            'lKnee':  self.lKnee,
            'rKnee':  self.rKnee,
            'lCalf':  self.lCalf,
            'rCalf':  self.rCalf,
            'lFoot':  self.lFoot,
            'rFoot':  self.rFoot,
        }

        # Build per-servo interpolation from current angle → rest angle
        trajectories = {}
        for name, servo in servos.items():
            try:
                current = servo.get_physical_angle()
            except ServoTimeoutError:
                self.get_logger().warn(f"{name} timed out reading angle, assuming rest position.")
                current = REST_POSITIONS[name]

            target = REST_POSITIONS[name]
            delta = target - current

            # interpolate_leg goes 0 → delta, so offset by current
            steps_arr = [current + v for v in interpolate_leg(delta, steps=steps, curve=curve)]
            trajectories[name] = steps_arr

        # Step through all trajectories in lockstep
        self.get_logger().info("Returning all servos to rest...")
        for i in range(steps):
            for name, servo in servos.items():
                try:
                    servo.move(trajectories[name][i])
                except ServoTimeoutError:
                    self.get_logger().error(f"Lost communication with {name} during return_to_rest.")
            time.sleep(step_delay)

        self.get_logger().info("All servos at rest position.")

    def test_legs(self, amplitude: float = 15.0, speed: float = 0.03):
        """
        Continuously oscillate both legs in a small, slow test movement around rest.
        Left and right legs move in opposite phase for a natural gait feel.

        Args:
            amplitude: Max degrees of movement from rest (default 8°)
            speed:     Delay between steps in seconds (larger = slower)
        """
        servos = {
            'lKnee': (self.lKnee, 100.0),
            'lCalf': (self.lCalf, 124.0),
            'lFoot': (self.lFoot, 106.0),
            'rKnee': (self.rKnee, 134.0),
            'rCalf': (self.rCalf, 100.0),
            'rFoot': (self.rFoot,  16.0),
        }

        self.get_logger().info("Starting both leg test. Ctrl+C to stop.")

        t = 0.0
        try:
            while True:
                l_wave = sin(t) * amplitude       # left leg phase
                r_wave = sin(t + pi) * amplitude  # right leg opposite phase

                phases = {
                    'lKnee': l_wave, 'lCalf': l_wave, 'lFoot': l_wave,
                    'rKnee': r_wave, 'rCalf': r_wave, 'rFoot': r_wave,
                }

                for name, (servo, rest) in servos.items():
                    try:
                        servo.move(rest + phases[name])
                    except ServoTimeoutError:
                        self.get_logger().error(f"Lost communication with {name} during leg test.")
                        return

                self.get_logger().info(
                    f"L offset: {l_wave:+.2f}°  R offset: {r_wave:+.2f}° | "
                    + "  ".join(f"{n}: {rest + phases[n]:.1f}°" for n, (_, rest) in servos.items())
                )

                t += 0.1
                time.sleep(speed)

        except KeyboardInterrupt:
            self.get_logger().info("Leg test stopped. Returning to rest.")
            self.return_to_rest()


def main(args=None):
    rclpy.init(args=args)

    node = ServoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()