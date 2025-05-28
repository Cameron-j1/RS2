#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from objectpose_msgs.msg import ObjectPose
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
from std_msgs.msg import Bool
from linkattacher_msgs.srv import AttachLink
from rclpy.task import Future
from linkattacher_msgs.srv import DetachLink
import time

class ChessBoardTracker(Node):
    def __init__(self):
        super().__init__('chess_board_tracker')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robotX = None
        self.robotY = None

        self.positions = {}

        piece_topics = [
            'black_bishop_c8', 'black_bishop_f8', 'black_king_e8', 'black_knight_b8', 'black_knight_g8',
            'black_pawn_a7', 'black_pawn_b7', 'black_pawn_c7', 'black_pawn_d7', 'black_pawn_e7',
            'black_pawn_f7', 'black_pawn_g7', 'black_pawn_h7', 'black_queen_d8', 'black_rook_a8', 'black_rook_h8',
            'white_bishop_c1', 'white_bishop_f1', 'white_king_e1', 'white_knight_b1', 'white_knight_g1',
            'white_pawn_a2', 'white_pawn_b2', 'white_pawn_c2', 'white_pawn_d2', 'white_pawn_e2',
            'white_pawn_f2', 'white_pawn_g2', 'white_pawn_h2', 'white_queen_d1', 'white_rook_a1', 'white_rook_h1'
        ]

        for piece in piece_topics:
            topic_name = f'/{piece}/ObjectPose'
            self.create_subscription(ObjectPose, topic_name, self.make_callback(piece), 10)

        self.create_subscription(
            Bool,
            '/servo_control',
            self.servo_control_callback,
            10
        )


        self.timer = self.create_timer(0.25, self.loop_callback)

    def calculate_distances(self, x, y):
        min_distance = 1000
        min_piece = ''
        for piece_name, (px, py) in self.positions.items():
            dist = math.sqrt((px - x)**2 + (py - y)**2)
            if(dist < min_distance):
                min_distance = dist
                min_piece = piece_name
        return min_piece

    def make_callback(self, piece_name):
        def callback(msg):
            self.positions[piece_name] = (msg.x, msg.y)
        return callback
    
    def attach_piece(self, model2_name):
        client = self.create_client(AttachLink, '/ATTACHLINK')
    
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/ATTACHLINK service not available, waiting...')
    
        request = AttachLink.Request()
        request.model1_name = 'ur'
        request.link1_name = 'wrist_3_link'
        request.model2_name = model2_name
        request.link2_name = 'link'
    
        future = client.call_async(request)
    
        # Optionally wait for result
        # rclpy.spin_until_future_complete(self, future)
    
        if future.result() is not None:
            self.get_logger().info(f"Successfully attached {model2_name}")
        else:
            self.get_logger().error(f"Failed to attach {model2_name}")

    def detach_piece(self, model2_name):
        client = self.create_client(DetachLink, '/DETACHLINK')
    
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/DETACHLINK service not available, waiting...')
    
        request = DetachLink.Request()
        request.model1_name = 'ur'
        request.link1_name = 'wrist_3_link'
        request.model2_name = model2_name
        request.link2_name = 'link'
    
        future = client.call_async(request)
    
        # Optionally wait for result
        # rclpy.spin_until_future_complete(self, future)
    
        if future.result() is not None:
            self.get_logger().info(f"Successfully detached {model2_name}")
        else:
            self.get_logger().error(f"Failed to detach {model2_name}")

    def servo_control_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Servo control: GRIP")
            self.attach_piece(self.closest_piece)
        else:
            self.get_logger().info("Servo control: RELEASE")
            self.detach_piece(self.closest_piece)
            
    def loop_callback(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',   # target frame
                'tool0',       # source frame
                rclpy.time.Time()  # latest available
            )
            self.robotX = trans.transform.translation.x
            self.robotY = trans.transform.translation.y
    
            # print(f"Tool0 relative to Base Link: x={self.robotX:.3f}, y={self.robotY:.3f}")
    
        except Exception as e:
            print(f"Could not transform base_link -> tool0: {str(e)}")

        try:
            self.closest_piece = self.calculate_distances(self.robotX, self.robotY)
        except Exception as e:
            self.get_logger().error(f"Could not calculate distances: {str(e)}")
            self.closest_piece = None

        # print(self.closest_piece)

        # for piece, (x, y) in self.positions.items():
            # self.get_logger().info(f"{piece}: x={x:.3f}, y={y:.3f}")



def main(args=None):
    time.sleep(5)
    rclpy.init(args=args)
    node = ChessBoardTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()