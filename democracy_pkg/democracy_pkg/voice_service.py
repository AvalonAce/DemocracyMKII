import rclpy
from rclpy.node import Node
import subprocess
import os
import queue
import threading
from std_msgs.msg import String
from std_srvs.srv import SetBool


class VoiceServer(Node):
    def __init__(self):
        super().__init__('voice_server')

        self.declare_parameter('audio_dir', '/home/byte_knight/ros2_ws/src/democracy_pkg/audio/voice_lines/')
        self.audio_dir = self.get_parameter('audio_dir').get_parameter_value().string_value

        self.enabled = True
        self.playingAudio = False

        # Audio queue
        self.audio_queue = queue.Queue()

        # Background thread to process the queue
        self.worker_thread = threading.Thread(target=self.process_queue, daemon=True)
        self.worker_thread.start()

        # Toggle service
        self.toggle_srv = self.create_service(
            SetBool,
            'toggle_voice',
            self.toggle_callback
        )

        # Voice request subscription
        self.subscription = self.create_subscription(
            String,
            'voice_request',
            self.play_audio_callback,
            10
        )

        self.get_logger().info('Voice server is ready. Currently ENABLED.')


    def toggle_callback(self, request, response):
        self.enabled = request.data
        state = "ENABLED" if self.enabled else "DISABLED"
        self.get_logger().info(f"Voice server {state}")
        response.success = True
        response.message = f"Voice server {state}"
        return response


    def play_audio_callback(self, msg):
        if not self.enabled:
            self.get_logger().warn("Voice is disabled, ignoring request.")
            return

        file_path = msg.data

        if not os.path.isabs(file_path):
            file_path = os.path.join(self.audio_dir, file_path)

        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return

        self.audio_queue.put(file_path)
        self.get_logger().info(f"Queued: {file_path} (queue size: {self.audio_queue.qsize()})")


    def process_queue(self):
        """Background thread — plays files from queue one by one."""
        while True:
            file_path = self.audio_queue.get()  # Blocks until item available

            self.playingAudio = True
            self.get_logger().info(f"Playing: {file_path}")

            try:
                subprocess.run(['mpg123', '-q', file_path], check=True)
                self.get_logger().info(f"Played: {file_path}")

            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Playback failed: {str(e)}")

            finally:
                self.playingAudio = False
                self.audio_queue.task_done()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()