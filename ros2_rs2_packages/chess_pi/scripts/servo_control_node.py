#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import pigpio
import time

# Constants
SERVO_PIN = 12
RELAY_PIN = 6
BUTTON_PIN = 4

STATE_TRUE = 1
STATE_FALSE = 0
DEBOUNCE_TIME_MS = 150  # debounce threshold in milliseconds

class Servo:
    def __init__(self, servo_pin, relay_pin):
        self.servo_pin = servo_pin
        self.relay_pin = relay_pin
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio")

        self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.relay_pin, pigpio.OUTPUT)
        self.pi.write(self.relay_pin, 0)

    def get_pi(self):
        return self.pi

    def set_state(self, state):
        self.pi.write(self.servo_pin, state)

    def power_on(self):
        self.pi.write(self.relay_pin, 1)

    def power_off(self):
        self.pi.write(self.relay_pin, 0)


class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.servo = Servo(SERVO_PIN, RELAY_PIN)
        self.pi = self.servo.get_pi()

        self.subscription = self.create_subscription(
            Bool,
            'servo_control',
            self.topic_callback,
            5
        )

        self.button_publisher = self.create_publisher(Bool, 'button_state', 5)

        self.pi.set_mode(BUTTON_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(BUTTON_PIN, pigpio.PUD_DOWN)

        self.last_button_event_time = 0

        self.cb = self.pi.callback(BUTTON_PIN, pigpio.EITHER_EDGE, self.button_callback)

        # Initialize outputs
        self.servo.power_off()
        self.servo.set_state(STATE_FALSE)

    def topic_callback(self, msg):
        print("Received data:", msg.data)
        self.servo.power_on()
        self.servo.set_state(STATE_TRUE if msg.data else STATE_FALSE)
        time.sleep(1)
        self.servo.power_off()

    def button_callback(self, gpio, level, tick):
        if level == pigpio.TIMEOUT:
            return

        current_time = time.time() * 1000
        if current_time - self.last_button_event_time < DEBOUNCE_TIME_MS:
            return

        self.last_button_event_time = current_time

        pressed = (level == 1)
        print("Button Pressed" if pressed else "Button Released")
        self.button_publisher.publish(Bool(data=pressed))


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ServoControlNode()
        print("Node is spinning")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        if 'node' in locals():
            node.servo.pi.stop()

if __name__ == '__main__':
    main()
