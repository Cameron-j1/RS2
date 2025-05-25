import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from queue import Queue
import threading
import time

import sounddevice as sd
from TTS.api import TTS


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            '/TTS',
            self.listener_callback,
            10)
        self.msg_queue = Queue()

        # Load Coqui TTS model
        self.get_logger().info("Loading TTS model...")
        self.tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC")
        self.get_logger().info("TTS model loaded.")

        sd.play(self.tts.tts("Commentator is ready", speed=1.5), samplerate=self.tts.synthesizer.output_sample_rate)
        sd.wait()

        # Start audio thread
        self.audio_thread = threading.Thread(target=self.audio_worker, daemon=True)
        self.audio_thread.start()


    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        self.msg_queue.put(msg.data)

    def audio_worker(self):
        while True:
            text = self.msg_queue.get()  # Blocking wait
            self.get_logger().info(f'Speaking: "{text}"')
            try:
                audio = self.tts.tts(text, speed=1.5)
                sd.play(audio, samplerate=self.tts.synthesizer.output_sample_rate)
                sd.wait()  # Wait for playback to finish
            except Exception as e:
                self.get_logger().error(f"Error during TTS: {e}")
            self.msg_queue.task_done()


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TTS node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
