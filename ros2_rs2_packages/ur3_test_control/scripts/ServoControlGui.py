import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tkinter as tk

class ServoControlGUI(Node):
    def __init__(self):
        super().__init__('servo_control_gui')
        self.publisher_ = self.create_publisher(Bool, 'servo_control', 10)
        self.init_gui()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Servo Control")

        self.label = tk.Label(self.root, text="Servo Control", font=("Arial", 16))
        self.label.pack(pady=10)

        self.on_button = tk.Button(self.root, text="ON", command=self.publish_on, bg="green", fg="white", width=10)
        self.on_button.pack(pady=5)

        self.off_button = tk.Button(self.root, text="OFF", command=self.publish_off, bg="red", fg="white", width=10)
        self.off_button.pack(pady=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def publish_on(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info("Published: ON")``

    def publish_off(self):
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)
        self.get_logger().info("Published: OFF")

    def on_close(self):
        self.get_logger().info("Shutting down GUI...")
        self.root.quit()  # Close the Tkinter window
        rclpy.shutdown()  # Shutdown the ROS 2 node

def main(args=None):
    rclpy.init(args=args)
    gui_node = ServoControlGUI()

    try:
        # Spin the ROS 2 node with Tkinter main loop running concurrently
        while rclpy.ok():
            gui_node.root.update_idletasks()  # Process any pending GUI events
            gui_node.root.update()  # Update the GUI
            rclpy.spin_once(gui_node)  # Process one ROS 2 callback
    except KeyboardInterrupt:
        gui_node.get_logger().info("Shutting down ROS 2 node...")
    finally:
        gui_node.destroy_node()

if __name__ == '__main__':
    main()
