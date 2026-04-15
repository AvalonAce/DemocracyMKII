import rclpy
from rclpy.node import Node
import pvporcupine
import pyaudio
import struct
import speech_recognition as sr
from vosk import Model, KaldiRecognizer
from std_srvs.srv import SetBool
import json
import random
import os
from dotenv import load_dotenv

from std_msgs.msg import String


class MicNode(Node):

    def __init__(self):
        super().__init__('mic_node')
        self.get_logger().info("====== MIC NODE STARTED ======")

        self.speech_pub = self.create_publisher(String, 'speech_input', 10)
        self.voice_request = self.create_publisher(String, 'voice_request', 10)
        self.movement_preset = self.create_publisher(String, 'movement_preset', 10)

        self.idle_client = self.create_client(SetBool, 'set_idle')

        # Load Vosk model
        model_path = '/home/byte_knight/ros2_ws/src/democracy_pkg/models/vosk-model-en'
        self.get_logger().info("Loading Vosk model...")
        self.vosk_model = Model(model_path)
        self.get_logger().info("Vosk model loaded.")

        # Porcupine wake word
        load_dotenv('/home/byte_knight/ros2_ws/src/.env')
        self.porcupine = pvporcupine.create(
            access_key = os.getenv('PORCUPINE_ACCESS_KEY'),
            keyword_paths=['/home/byte_knight/ros2_ws/src/democracy_pkg/wake_words/Pilot_en_raspberry-pi_v4_0_0.ppn']
        )

        # PyAudio setup
        self.pa = pyaudio.PyAudio()
        self.audio_stream = self.pa.open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length
        )

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.capture_duration = 5

        self.get_logger().info("Listening for wake word 'Pilot'...")
        self.timer = self.create_timer(0.01, self.listen_loop)


    def listen_loop(self):
        try:
            pcm = self.audio_stream.read(
                self.porcupine.frame_length,
                exception_on_overflow=False
            )
            pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

            keyword_index = self.porcupine.process(pcm)

            if keyword_index >= 0:
                self.get_logger().info("Command detected!")
                self.categorize_and_ship("uhh") 
                self.capture_speech()

        except Exception as e:
            self.get_logger().error(f"Listen loop error: {str(e)}")


    def capture_speech(self):
        self.get_logger().info(f"Capturing speech for {self.capture_duration} seconds...")
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                audio = self.recognizer.listen(
                    source,
                    timeout=self.capture_duration,
                    phrase_time_limit=self.capture_duration
                )
            self.get_logger().info("Speech captured, processing...")
            self.process_speech(audio)

        except sr.WaitTimeoutError:
            self.get_logger().warn("No speech detected within timeout.")
        except Exception as e:
            self.get_logger().error(f"Capture error: {str(e)}")


    def process_speech(self, audio):
        """Offline STT using Vosk."""
        try:
            raw_data = audio.get_raw_data(convert_rate=16000, convert_width=2)

            rec = KaldiRecognizer(self.vosk_model, 16000)
            rec.AcceptWaveform(raw_data)

            result = json.loads(rec.FinalResult())
            text = result.get('text', '').strip()

            if text:
                self.get_logger().info(f"Recognized: {text}")
                msg = String()
                msg.data = text
                self.categorize_and_ship(text)
            else:
                self.get_logger().warn("Could not understand audio.")

        except Exception as e:
            self.get_logger().error(f"STT error: {str(e)}")


    def categorize_and_ship(self, text):
        text = text.lower()

        # Format: 'keyword(s)': ('audio_prefix', min_variation, max_variation)
        keyword_map = {
            'dance':            ('bust',       1, 4),
            'are you':      ('reporting',    1, 2),
            'regard':           ('sergeant',      1, 2),
            'line':             ('ddmf',        1, 3),
            'thing':        ('demo',        1, 3),
            'nominal':          ('nomi', 1, 2),
            'shutdown':      ('off', 1, 2),
            'stop':             ('capitain', 1, 4),
            'robot':            ('cava', 1, 3),
            'uhh':              ('uhh', 1, 4),
            'error':          ('error', 1, 2),
            'robot':            ('cava', 1, 3),
            'program':          ('town', 1, 2),
            'guard':            ('capitain', 1, 4),
            'stand':            ('capitain', 1, 4),


            'think':            ('think', 1, 4),
            'are you':          ('reporting', 1, 2),
            'feel':             ('feel', 1, 4),
            'know':             ('know', 1, 4),
            'best':             ('best', 1, 4),
        }

        for keyword, (prefix, min_var, max_var) in keyword_map.items():
            if keyword in text:
                variation = random.randint(min_var, max_var)
                filename = f"{prefix}_{variation}_fr.mp3"

                self.get_logger().info(f"Keyword '{keyword}' matched → publishing: {filename}")

                # Keyword Movement Triggers
                if keyword == 'are you' or keyword == 'regard':
                    self.get_logger().info("Triggering 'reporting' maneuver")
                    self.movement_preset.publish(String(data="salute"))
                elif keyword == 'line':
                    self.get_logger().info("Triggering 'ddmf' maneuver")
                    self.movement_preset.publish(String(data="ddmf"))
                elif keyword == 'thing':
                    self.get_logger().info("Triggering 'answer' maneuver")
                    self.movement_preset.publish(String(data="answer"))
                elif keyword == 'shutdown':
                    self.get_logger().info("Shutdown Sequence Initiated")
                    self.movement_preset.publish(String(data="shutdown"))
                elif keyword == 'stop':
                    self.get_logger().info("Stop Command Detected")
                    self.movement_preset.publish(String(data="stop"))
                elif keyword == 'robot':
                    self.get_logger().info("Triggering 'fawn' maneuver")
                    self.movement_preset.publish(String(data="fawn"))
                elif keyword == 'program':
                    self.get_logger().info("Triggering 'answer' maneuver")
                    self.movement_preset.publish(String(data="answer"))
                elif keyword == 'guard':
                    self.get_logger().info("Triggering 'guard' maneuver")
                    self.movement_preset.publish(String(data="guard"))
                    self.change_idle(state=True)
                elif keyword == 'stand':
                    self.get_logger().info("Stopping idle")
                    self.change_idle(state=False)

                # End conditional
                if keyword in ['think', 'feel', 'know', 'best']:
                    random_response = random.randint(1, 2)

                    if random_response == 1:
                        self.get_logger().info("Disagreeing...")
                        random_response = random.randint(1, 2)
                        if random_response == 1:
                            random_response = random.randint(1, 2)
                            filename = f"chief_{random_response}_fr.mp3"
                            self.get_logger().info(f"Publishing disagreement response: {filename}")
                            self.voice_request.publish(String(data=filename))
                        else:
                            filename = f"negative_{random.randint(1, 4)}_fr.mp3"
                            self.get_logger().info(f"Publishing disagreement response: {filename}")
                            self.voice_request.publish(String(data=filename))
                    else:
                        self.get_logger().info("Agreeing...")
                        random_response = random.randint(1, 2)
                        if random_response == 1:
                            filename = f"capitain_{random.randint(1, 4)}_fr.mp3"
                            self.get_logger().info(f"Publishing agreement response: {filename}")
                            self.voice_request.publish(String(data=filename))
                        else:
                            filename = f"hella_{random.randint(1, 3)}_fr.mp3"
                            self.get_logger().info(f"Publishing agreement response: {filename}")
                            self.voice_request.publish(String(data=filename))



                    return
                

                msg = String()
                msg.data = filename
                self.voice_request.publish(msg)

                if keyword == "dance":
                    self.get_logger().info("Triggering 'dance' maneuver")
                    self.voice_request.publish(String(data="hungry_like_a_wolf.mp3"))
                    self.movement_preset.publish(String(data="dance"))

                return  # Remove return if you want multiple keywords to trigger

        self.get_logger().warn(f"No keywords matched in: '{text}'")


    def change_idle(self, state: bool = False):
        # Wait for the service to be available
        if not self.idle_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("set_idle service not available")
            return

        request = SetBool.Request()
        request.data = state  # Turn off on_watch

        future = self.idle_client.call_async(request)
        future.add_done_callback(self._idle_response_callback)


    def _idle_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"set_idle response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"set_idle call failed: {e}")


    def destroy_node(self):
        self.get_logger().info("Shutting down mic node...")
        self.audio_stream.stop_stream()
        self.audio_stream.close()
        self.pa.terminate()
        self.porcupine.delete()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()