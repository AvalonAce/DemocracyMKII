#!/usr/bin/env python3
"""
ROS2 Node: stepper_motor_node
Hardware: 2x HiLetgo ULN2003 + 28BYJ-48 4-phase stepper motors
Platform: Raspberry Pi 4B (Ubuntu 22.04)
GPIO backend: lgpio (built from source)

Subscribed Topics:
  /stepper/command  (stepper_msgs/StepperCommand) — move one or both motors

  StepperCommand fields:
    uint8   motor_id    — 1 or 2 (0 = both simultaneously)
    float32 angle_deg   — degrees to rotate (positive=CW, negative=CCW)
    uint8   speed       — 0=SLOW, 1=MEDIUM, 2=FAST  (see speed table below)

Speed table (step_delay):
  SLOW   = 5.0 ms  (~12 rpm)  — precise, high torque
  MEDIUM = 2.0 ms  (~30 rpm)  — balanced
  FAST   = 1.0 ms  (~60 rpm)  — max reliable speed for 28BYJ-48

Published Topics:
  /stepper/motor1/current_angle  (std_msgs/Float32) — cumulative angle in degrees
  /stepper/motor2/current_angle  (std_msgs/Float32)
  /stepper/motor1/busy           (std_msgs/Bool)
  /stepper/motor2/busy           (std_msgs/Bool)

Parameters:
  motor1_in1..4  (int) — BCM pins motor 1 (defaults: 5,6,13,19)
  motor2_in1..4  (int) — BCM pins motor 2 (defaults: 16,20,21,26)
  steps_per_rev  (int, default 4096)
  coil_test      (bool, default false)
  sequence_test  (bool, default false)
  self_test      (bool, default false)

Example publish:
  ros2 topic pub --once /stepper/command stepper_msgs/msg/StepperCommand \
    "{motor_id: 1, angle_deg: 90.0, speed: 1}"
"""

import threading
import time

import lgpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from stepper_msgs.msg import StepperCommand

# ── Speed table (step delay in seconds) ──────────────────────────────────────
SPEED_SLOW   = 0.005   # 5.0 ms — ~12 rpm, high torque
SPEED_MEDIUM = 0.002   # 2.0 ms — ~30 rpm, balanced
SPEED_FAST   = 0.001   # 1.0 ms — ~60 rpm, max reliable for 28BYJ-48

SPEED_MAP = {
    StepperCommand.SLOW:   SPEED_SLOW,
    StepperCommand.MEDIUM: SPEED_MEDIUM,
    StepperCommand.FAST:   SPEED_FAST,
}

HALF_STEP_SEQUENCE = [
    (1, 0, 0, 0),
    (1, 1, 0, 0),
    (0, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 0),
    (0, 0, 1, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
]
SEQ_LEN = len(HALF_STEP_SEQUENCE)


class StepperMotor:
    """Low-level lgpio interface for one 28BYJ-48."""

    def __init__(self, chip_handle: int, pins: list[int]):
        self._h    = chip_handle
        self._pins = pins
        self._seq  = 0
        for pin in self._pins:
            lgpio.gpio_claim_output(self._h, pin, 0)

    def step(self, direction: int):
        self._seq = (self._seq + direction) % SEQ_LEN
        pattern   = HALF_STEP_SEQUENCE[self._seq]
        for pin, val in zip(self._pins, pattern):
            lgpio.gpio_write(self._h, pin, val)

    def release(self):
        for pin in self._pins:
            lgpio.gpio_write(self._h, pin, 0)

    def free(self):
        self.release()
        for pin in self._pins:
            lgpio.gpio_free(self._h, pin)


class MotorController:
    """Thread-safe move controller for one StepperMotor."""

    def __init__(self, motor_id: int, motor: StepperMotor,
                 steps_per_rev: int, logger):
        self._id            = motor_id
        self._motor         = motor
        self._steps_per_rev = steps_per_rev
        self._logger        = logger

        self._total_steps   = 0          # cumulative steps (used for angle tracking)
        self._stop_event    = threading.Event()
        self._move_thread: threading.Thread | None = None
        self._lock          = threading.Lock()

    @property
    def current_angle_deg(self) -> float:
        return self._total_steps / self._steps_per_rev * 360.0

    @property
    def busy(self) -> bool:
        return self._move_thread is not None and self._move_thread.is_alive()

    def command_move(self, angle_deg: float, speed: int):
        steps = int(round(abs(angle_deg) / 360.0 * self._steps_per_rev))
        if steps == 0:
            return

        direction  = 1 if angle_deg > 0 else -1
        step_delay = SPEED_MAP.get(speed, SPEED_MEDIUM)

        self._logger.info(
            f'Motor {self._id}: {angle_deg:+.1f}° → {steps} steps '
            f'({"CW" if direction > 0 else "CCW"}, '
            f'{"SLOW" if speed == StepperCommand.SLOW else "FAST" if speed == StepperCommand.FAST else "MEDIUM"})'
        )

        with self._lock:
            self._stop_event.set()
            if self._move_thread and self._move_thread.is_alive():
                self._move_thread.join(timeout=2.0)
            self._stop_event.clear()
            self._move_thread = threading.Thread(
                target=self._move,
                args=(steps, direction, step_delay),
                daemon=True,
            )
            self._move_thread.start()

    def blocking_move(self, angle_deg: float, speed: int):
        done = threading.Event()
        steps      = int(round(abs(angle_deg) / 360.0 * self._steps_per_rev))
        direction  = 1 if angle_deg > 0 else -1
        step_delay = SPEED_MAP.get(speed, SPEED_MEDIUM)

        def _inner():
            self._move(steps, direction, step_delay)
            done.set()

        with self._lock:
            self._stop_event.set()
            if self._move_thread and self._move_thread.is_alive():
                self._move_thread.join(timeout=2.0)
            self._stop_event.clear()
            self._move_thread = threading.Thread(target=_inner, daemon=True)
            self._move_thread.start()

        done.wait()

    def stop(self):
        self._stop_event.set()

    def _move(self, steps: int, direction: int, step_delay: float):
        remaining = steps
        while remaining > 0:
            if self._stop_event.is_set():
                self._logger.info(f'Motor {self._id}: move interrupted.')
                break
            self._motor.step(direction)
            self._total_steps += direction
            remaining         -= 1
            time.sleep(step_delay)
        self._motor.release()
        self._logger.info(
            f'Motor {self._id}: done. '
            f'Cumulative angle: {self.current_angle_deg:.2f}°'
        )

    def shutdown(self):
        self._stop_event.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=2.0)
        self._motor.free()


class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('motor1_in1',    5)
        self.declare_parameter('motor1_in2',    6)
        self.declare_parameter('motor1_in3',   13)
        self.declare_parameter('motor1_in4',   19)
        self.declare_parameter('motor2_in1',   16)
        self.declare_parameter('motor2_in2',   20)
        self.declare_parameter('motor2_in3',   21)
        self.declare_parameter('motor2_in4',   26)
        self.declare_parameter('steps_per_rev', 4096)
        self.declare_parameter('coil_test',     False)
        self.declare_parameter('sequence_test', False)
        self.declare_parameter('self_test',     False)

        m1_pins       = [self.get_parameter(f'motor1_in{i}').value for i in range(1, 5)]
        m2_pins       = [self.get_parameter(f'motor2_in{i}').value for i in range(1, 5)]
        steps_per_rev = self.get_parameter('steps_per_rev').value

        # ── lgpio ────────────────────────────────────────────────────────
        self._chip = lgpio.gpiochip_open(0)
        self.get_logger().info('lgpio chip opened OK')

        self._controllers = [
            MotorController(1, StepperMotor(self._chip, m1_pins), steps_per_rev, self.get_logger()),
            MotorController(2, StepperMotor(self._chip, m2_pins), steps_per_rev, self.get_logger()),
        ]

        self.get_logger().info(
            f'Motor 1 pins: {m1_pins} | Motor 2 pins: {m2_pins} | '
            f'{steps_per_rev} steps/rev'
        )

        # ── Command subscriber ────────────────────────────────────────────
        self.create_subscription(
            StepperCommand, '/stepper/command', self._cb_command, 10
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_angle = [
            self.create_publisher(Float32, f'/stepper/motor{i}/current_angle', 10)
            for i in range(1, 3)
        ]
        self._pub_busy = [
            self.create_publisher(Bool, f'/stepper/motor{i}/busy', 10)
            for i in range(1, 3)
        ]

        self.create_timer(0.1, self._publish_status)

        # ── Startup tests ─────────────────────────────────────────────────
        if any([
            self.get_parameter('coil_test').value,
            self.get_parameter('sequence_test').value,
            self.get_parameter('self_test').value,
        ]):
            self._startup_timer = self.create_timer(1.0, self._run_startup_test)

        self.get_logger().info(
            'stepper_motor_node ready.\n'
            '  Publish to /stepper/command (StepperCommand):\n'
            '    motor_id  : 1, 2, or 0 (both)\n'
            '    angle_deg : degrees to rotate (+CW / -CCW)\n'
            '    speed     : 0=SLOW, 1=MEDIUM, 2=FAST'
        )

    # ────────────────────────────────────────────────────────────────────
    # Command callback
    # ────────────────────────────────────────────────────────────────────

    def _cb_command(self, msg: StepperCommand):
        if msg.motor_id == 0:
            # Both motors simultaneously
            for ctrl in self._controllers:
                ctrl.command_move(msg.angle_deg, msg.speed)
        elif msg.motor_id in (1, 2):
            self._controllers[msg.motor_id - 1].command_move(msg.angle_deg, msg.speed)
        else:
            self.get_logger().warn(
                f'Unknown motor_id {msg.motor_id}. Use 1, 2, or 0 (both).'
            )

    # ────────────────────────────────────────────────────────────────────
    # Status publisher
    # ────────────────────────────────────────────────────────────────────

    def _publish_status(self):
        for idx, ctrl in enumerate(self._controllers):
            a      = Float32()
            a.data = ctrl.current_angle_deg
            self._pub_angle[idx].publish(a)

            b      = Bool()
            b.data = ctrl.busy
            self._pub_busy[idx].publish(b)

    # ────────────────────────────────────────────────────────────────────
    # Startup tests
    # ────────────────────────────────────────────────────────────────────

    def _run_startup_test(self):
        self._startup_timer.cancel()
        if self.get_parameter('coil_test').value:
            t = threading.Thread(target=self._coil_test, daemon=True)
        elif self.get_parameter('sequence_test').value:
            t = threading.Thread(target=self._sequence_test, daemon=True)
        else:
            t = threading.Thread(target=self._self_test, daemon=True)
        t.start()

    def _coil_test(self):
        self.get_logger().info('COIL TEST — each coil ON for 1 s, both motors')
        single_coil_steps = [0, 2, 4, 6]  # indices in HALF_STEP_SEQUENCE with 1 coil
        for idx, ctrl in enumerate(self._controllers):
            self.get_logger().info(f'--- Motor {idx + 1} ---')
            for i, seq_idx in enumerate(single_coil_steps):
                self.get_logger().info(f'  IN{i+1} ON…')
                pattern = HALF_STEP_SEQUENCE[seq_idx]
                for pin, val in zip(ctrl._motor._pins, pattern):
                    lgpio.gpio_write(self._chip, pin, val)
                time.sleep(1.0)
                ctrl._motor.release()
                time.sleep(0.2)
        self.get_logger().info('COIL TEST complete.')

    def _sequence_test(self):
        self.get_logger().info('SEQUENCE TEST — 16 slow steps, both motors')
        for step in range(16):
            for ctrl in self._controllers:
                ctrl._motor.step(1)
            self.get_logger().info(f'  Step {step+1:>2}/16')
            time.sleep(0.5)
        for ctrl in self._controllers:
            ctrl._motor.release()
        self.get_logger().info('SEQUENCE TEST complete.')

    def _self_test(self):
        self.get_logger().info('SELF-TEST — CW 360° MEDIUM → pause → CCW 360° MEDIUM')
        for idx, ctrl in enumerate(self._controllers):
            mid = idx + 1
            self.get_logger().info(f'Motor {mid} — CW 360°…')
            ctrl.blocking_move(360.0, StepperCommand.MEDIUM)
            time.sleep(1.0)
            self.get_logger().info(f'Motor {mid} — CCW 360°…')
            ctrl.blocking_move(-360.0, StepperCommand.MEDIUM)
            self.get_logger().info(f'Motor {mid} — done ✓')
        self.get_logger().info('SELF-TEST complete.')

    # ────────────────────────────────────────────────────────────────────
    # Cleanup
    # ────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        for ctrl in self._controllers:
            ctrl.shutdown()
        lgpio.gpiochip_close(self._chip)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()