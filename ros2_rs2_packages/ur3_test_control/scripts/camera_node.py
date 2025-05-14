#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from collections import defaultdict
import cv2
from inference import get_model
from std_msgs.msg import Bool

class CameraNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Topic for the image stream
            self.image_callback,
            10
        )

        self.get_logger().info("Image subscriber created")
        self.chess_subscription = self.create_subscription(
            String,
            '/chess_moves',
            self.chess_callback,
            10)
        
        self.get_logger().info("Chess subscripber created")
    
        self.player_publish = self.create_publisher(
            String,
            '/player_move',
            10
        )
        self.get_logger().info("Player move publisher created")

        self.image_action_subscription = self.create_subscription(
            Bool,
            '/take_image',
            self.img_action,
            10)


        self.bridge = CvBridge()
        self.latest_image = None
        self.get_logger().info('Node started. Displaying image stream. Press Ctrl+C to stop.')
        self.board = [['-'] * 8 for _ in range(8)]
        self.is_castling = [False] * 64
        self.model = get_model(model_id="reddotdetection/1")
        self.takeImg = False

    def img_action(self, msg):
        self.takeImg = msg.data

    def chess_callback(self, msg):
        print("Msg len")
        print(len(msg.data))
        if len(msg.data) > 50:
            for i in range(8):
                for j in range(8):
                    self.board[i][j] = msg.data[i * 8 + j]
            self.get_logger().info('Board received:')
            for row in self.board:
                self.get_logger().info(' '.join(row))

    def image_callback(self, msg):
        try:
            if self.takeImg:
                # Convert ROS Image message to OpenCV format (BGR)
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_image = self.make_square_crop(self.latest_image)
                # # When the spacebar is pressed, it will trigger the chess analysis process (TODO)
                chessBoardB = self.process_chessboard(self.latest_image)
                chessStr = np.array2string(chessBoardB, separator=', ')
                self.get_logger().info(f'Occupancy grid from black perspective:\n{chessStr}')
                chessBoardW = [row[::-1] for row in chessBoardB[::-1]]
                # chessStr = np.array2string(chessBoardW, separator=', ')
                # self.get_logger().info(f'Occupancy grid from white perspective:\n{chessStr}')
                # # Convert to the white perspective to match with the chess node

                stop = False
                for i in range(8):
                    for y in range(8):
                        # the piece has moved
                        if chessBoardW[i][y] == 1 and self.board[i][y] != '-':
                            cellToCheck = self.get_possible_moves(i, y)
                            print(cellToCheck)
                            for j in range(len(cellToCheck)):
                                if chessBoardW[cellToCheck[j][0]][cellToCheck[j][1]] == 0:
                                    pieceInitialPos = [i, y]
                                    pieceFinalPos = cellToCheck[j]
                                    print("pieceInitialPos")
                                    print(pieceInitialPos)
                                    print("pieceFinalPos")
                                    print(pieceFinalPos)
                                    playerMove = [pieceInitialPos, pieceFinalPos]
                                    stop = True
                                    break
                        if stop:
                            break
                    if stop:
                        break
                self.get_logger().info(f'Start P: {playerMove[0][0]}, {playerMove[0][1]}; End P: {playerMove[1][0]}, {playerMove[1][1]} ')
                # # Publish the move to the chess node for stockfish
                toChessNode = ''.join(str(num) for sublist in playerMove for num in sublist)
                self.player_publish.publish(String(data=toChessNode))
                # Display the image in a window
                if self.latest_image is not None:
                    cv2.imshow('Camera Stream', self.latest_image)
                    cv2.waitKey(1)  # Refresh the window (1ms delay)
                else:
                    self.get_logger().warn('No image data received yet.')
                self.takeImg = False
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def isDot(self, img):
        prediction = self.model.infer(img)[0]
        return prediction.predictions[0].class_id

    def find_move(self, newOccupancy):
        # Heuristic check for castling
        if newOccupancy[7][0] == 0 and newOccupancy[7][4] == 0 and self.board[7][0] != '-' and self.board[7][4] != '-':
            self.get_logger().info("White Queen-Side Castling")
            return [[7, 4], [7, 2], [7, 0], [7, 3]]

        if newOccupancy[7][7] == 0 and newOccupancy[7][4] == 0 and self.board[7][7] != '-' and self.board[7][4] != '-':
            self.get_logger().info("White King-Side Castling")
            return [[7, 4], [7, 6], [7, 7], [7, 5]]

        for i in range(8):
            for y in range(8):
                # the piece has moved
                if newOccupancy[i][y] == 1 and self.board[i][y] != '-':
                    cellToCheck = self.get_possible_moves(i, y)
                    self.get_logger().info(f'Moves:\n{len(cellToCheck)}')
                    for j in range(len(cellToCheck)):
                        if newOccupancy[cellToCheck[j][0]][cellToCheck[j][1]] == 0:
                            pieceInitialPos = [i, y]
                            pieceFinalPos = cellToCheck[j]
                            return [pieceInitialPos, pieceFinalPos]
        return None

    def make_square_crop(self, img):
        """Crop the image to a square by trimming the longer dimension."""
        height, width = img.shape[:2]
        size = min(height, width)  # Use the smaller dimension as the square size
        start_h = (height - size) // 2  # Center the crop vertically
        start_w = (width - size) // 2   # Center the crop horizontally
        square_img = img[start_h:start_h + size, start_w:start_w + size]
        return square_img
    
    def is_valid(self, row, col):
        return 0 <= row < 8 and 0 <= col < 8

    # Helper function to check if two pieces are enemies
    def is_enemy(self, piece1, piece2):
        return (piece1.isupper() and piece2.islower()) or (piece1.islower() and piece2.isupper())

    def get_possible_moves(self, row, col):
        moves = []
        current_piece = self.board[row][col]
        is_white = current_piece.isupper()

        if not self.is_valid(row, col) or current_piece == '-':
            return moves  # Invalid position or empty cell

        if current_piece in ('p', 'P'):
            direction = -1 if is_white else 1  # White moves up, Black moves down
            start_row = 6 if is_white else 1   # Starting row for two-step move

            # Forward move
            new_row = row + direction
            if self.is_valid(new_row, col) and self.board[new_row][col] == '-':
                moves.append([new_row, col])
                # Two-step move from starting row
                if row == start_row and self.board[new_row + direction][col] == '-':
                    moves.append([new_row + direction, col])

            # Capture moves (diagonal)
            for dc in [-1, 1]:
                new_row = row + direction
                new_col = col + dc
                if (self.is_valid(new_row, new_col) and self.board[new_row][new_col] != '-' and 
                    self.is_enemy(current_piece, self.board[new_row][new_col])):
                    moves.append([new_row, new_col])

        elif current_piece in ('r', 'R'):
            # Directions: up, down, left, right
            directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]
            for dir in directions:
                new_row, new_col = row, col
                while True:
                    new_row += dir[0]
                    new_col += dir[1]
                    if not self.is_valid(new_row, new_col):
                        break
                    if self.board[new_row][new_col] == '-':
                        moves.append([new_row, new_col])
                    elif self.is_enemy(current_piece, self.board[new_row][new_col]):
                        moves.append([new_row, new_col])
                        break  # Stop after capturing
                    else:
                        break  # Blocked by friendly piece

        elif current_piece in ('n', 'N'):
            offsets = [
                [-2, -1], [-2, 1], [-1, -2], [-1, 2],
                [1, -2], [1, 2], [2, -1], [2, 1]
            ]
            for offset in offsets:
                new_row = row + offset[0]
                new_col = col + offset[1]
                if (self.is_valid(new_row, new_col) and 
                    (self.board[new_row][new_col] == '-' or self.is_enemy(current_piece, self.board[new_row][new_col]))):
                    moves.append([new_row, new_col])

        elif current_piece in ('b', 'B'):
            directions = [[-1, -1], [-1, 1], [1, -1], [1, 1]]
            for dir in directions:
                new_row, new_col = row, col
                while True:
                    new_row += dir[0]
                    new_col += dir[1]
                    if not self.is_valid(new_row, new_col):
                        break
                    if self.board[new_row][new_col] == '-':
                        moves.append([new_row, new_col])
                    elif self.is_enemy(current_piece, self.board[new_row][new_col]):
                        moves.append([new_row, new_col])
                        break
                    else:
                        break

        elif current_piece in ('q', 'Q'):
            directions = [
                [-1, 0], [1, 0], [0, -1], [0, 1],  # Rook-like
                [-1, -1], [-1, 1], [1, -1], [1, 1]  # Bishop-like
            ]
            for dir in directions:
                new_row, new_col = row, col
                while True:
                    new_row += dir[0]
                    new_col += dir[1]
                    if not self.is_valid(new_row, new_col):
                        break
                    if self.board[new_row][new_col] == '-':
                        moves.append([new_row, new_col])
                    elif self.is_enemy(current_piece, self.board[new_row][new_col]):
                        moves.append([new_row, new_col])
                        break
                    else:
                        break

        elif current_piece in ('k', 'K'):
            # Standard king moves
            offsets = [
                [-1, -1], [-1, 0], [-1, 1],
                [0, -1],           [0, 1],
                [1, -1],  [1, 0],  [1, 1]
            ]
            for offset in offsets:
                new_row = row + offset[0]
                new_col = col + offset[1]
                if (self.is_valid(new_row, new_col) and 
                    (self.board[new_row][new_col] == '-' or self.is_enemy(current_piece, self.board[new_row][new_col]))):
                    moves.append([new_row, new_col])

            # Castling moves
            is_white = (current_piece == 'K')
            start_row = 7 if is_white else 0  # White king at row 7, Black at row 0

            if row == start_row and col == 4:  # King at e1 (7,4) or e8 (0,4)
                # Kingside castling (right, toward h-file)
                if (self.board[start_row][7] == ('R' if is_white else 'r') and  # Rook at h1/h8
                    self.board[start_row][5] == '-' and self.board[start_row][6] == '-'):  # f and g empty
                    moves.append([start_row, 6])  # King moves to g1/g8
                    self.is_castling[start_row * 8 + 6] = True
                # Queenside castling (left, toward a-file)
                if (self.board[start_row][0] == ('R' if is_white else 'r') and  # Rook at a1/a8
                    self.board[start_row][1] == '-' and self.board[start_row][2] == '-' and self.board[start_row][3] == '-'):  # b, c, d empty
                    moves.append([start_row, 2])  # King moves to c1/c8
                    self.is_castling[start_row * 8 + 2] = True

        return moves

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Image Publisher Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()