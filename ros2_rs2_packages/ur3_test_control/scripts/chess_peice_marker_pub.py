#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from objectpose_msgs.msg import ObjectPose
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import std_msgs.msg

class ChessPieceVisualizer(Node):
    def __init__(self):
        super().__init__('chess_piece_visualizer')
        
        # Dictionary to store the latest pose for each chess piece
        self.chess_pieces = {}
        
        # Define chess piece topics
        chess_piece_topics = [
            # White pieces
            '/white_bishop_c1/ObjectPose',
            '/white_bishop_f1/ObjectPose',
            '/white_king_e1/ObjectPose',
            '/white_knight_b1/ObjectPose',
            '/white_knight_g1/ObjectPose',
            '/white_pawn_a2/ObjectPose',
            '/white_pawn_b2/ObjectPose',
            '/white_pawn_c2/ObjectPose',
            '/white_pawn_d2/ObjectPose',
            '/white_pawn_e2/ObjectPose',
            '/white_pawn_f2/ObjectPose',
            '/white_pawn_g2/ObjectPose',
            '/white_pawn_h2/ObjectPose',
            '/white_queen_d1/ObjectPose',
            '/white_rook_a1/ObjectPose',
            '/white_rook_h1/ObjectPose',
            # Black pieces
            '/black_bishop_c8/ObjectPose',
            '/black_bishop_f8/ObjectPose',
            '/black_king_e8/ObjectPose',
            '/black_knight_b8/ObjectPose',
            '/black_knight_g8/ObjectPose',
            '/black_pawn_a7/ObjectPose',
            '/black_pawn_b7/ObjectPose',
            '/black_pawn_c7/ObjectPose',
            '/black_pawn_d7/ObjectPose',
            '/black_pawn_e7/ObjectPose',
            '/black_pawn_f7/ObjectPose',
            '/black_pawn_g7/ObjectPose',
            '/black_pawn_h7/ObjectPose',
            '/black_queen_d8/ObjectPose',
            '/black_rook_a8/ObjectPose',
            '/black_rook_h8/ObjectPose'
        ]
        
        # Create subscribers for each chess piece
        for topic in chess_piece_topics:
            piece_name = topic.split('/')[1]
            self.create_subscription(
                ObjectPose,
                topic,
                lambda msg, piece=piece_name: self.pose_callback(msg, piece),
                10
            )
        
        # Create publisher for marker array
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/chess_pieces/visualization',
            10
        )
        
        # Timer for publishing marker array
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info('Chess Piece Visualizer started')
    
    def pose_callback(self, msg, piece_name):
        """Callback function for chess piece pose messages"""
        self.chess_pieces[piece_name] = msg
        self.get_logger().debug(f'Received pose for {piece_name}')
    
    def publish_markers(self):
        """Create and publish marker array for visualization"""
        marker_array = MarkerArray()
        
        for i, (piece_name, pose) in enumerate(self.chess_pieces.items()):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "world"  # Adjust frame ID if needed
            
            marker.ns = "chess_pieces"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set marker position from ObjectPose fields
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            marker.pose.position.z = pose.z
            marker.pose.orientation.x = pose.qx
            marker.pose.orientation.y = pose.qy
            marker.pose.orientation.z = pose.qz
            marker.pose.orientation.w = pose.qw
            
            # Set marker size (small sphere)
            marker.scale.x = 0.02  # 5 cm diameter
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            
            # Set marker color based on piece type
            if "black" in piece_name:
                # Black pieces are dark-colored
                if "pawn" in piece_name:
                    marker.color.r = 0.3
                    marker.color.g = 0.3
                    marker.color.b = 0.3
                elif "knight" in piece_name:
                    marker.color.r = 0.2
                    marker.color.g = 0.3
                    marker.color.b = 0.2
                elif "bishop" in piece_name:
                    marker.color.r = 0.2
                    marker.color.g = 0.2
                    marker.color.b = 0.3
                elif "rook" in piece_name:
                    marker.color.r = 0.3
                    marker.color.g = 0.2
                    marker.color.b = 0.2
                elif "queen" in piece_name:
                    marker.color.r = 0.3
                    marker.color.g = 0.1
                    marker.color.b = 0.3
                elif "king" in piece_name:
                    marker.color.r = 0.3
                    marker.color.g = 0.3
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.3
                    marker.color.g = 0.3
                    marker.color.b = 0.3
            else:
                # White pieces are light-colored
                if "pawn" in piece_name:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.8
                elif "knight" in piece_name:
                    marker.color.r = 0.8
                    marker.color.g = 1.0
                    marker.color.b = 0.8
                elif "bishop" in piece_name:
                    marker.color.r = 0.8
                    marker.color.g = 0.8
                    marker.color.b = 1.0
                elif "rook" in piece_name:
                    marker.color.r = 1.0
                    marker.color.g = 0.8
                    marker.color.b = 0.8
                elif "queen" in piece_name:
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 1.0
                elif "king" in piece_name:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
            
            marker.color.a = 1.0  # Full opacity
            
            # Set lifetime of marker
            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            
            marker_array.markers.append(marker)
        
        # Publish marker array if we have markers
        if marker_array.markers:
            self.marker_publisher.publish(marker_array)
            self.get_logger().debug(f'Published {len(marker_array.markers)} markers')

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = ChessPieceVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        visualizer.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()