import json
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

SERIAL_PORT = '/dev/ttyACM1'  # data port, not REPL
BAUD_RATE   = 14337


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu')
        self.get_logger().info("====== IMU NODE STARTING ======")

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"  Connected to Trinkey on {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"  Failed to open serial port: {e}")
            raise

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.create_timer(0.01, self.publish_imu)


    def publish_imu(self):
        if not self.ser.in_waiting:
            return

        try:
            line = self.ser.readline().decode().strip()
            data = json.loads(line)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return  # skip malformed packets

        ax, ay, az = data["a"]
        gx, gy, gz = data["g"]

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx * 0.0174533  # deg/s → rad/s
        msg.angular_velocity.y = gy * 0.0174533
        msg.angular_velocity.z = gz * 0.0174533

        msg.orientation_covariance[0] = -1  # no orientation data

        self.imu_pub.publish(msg)


    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()