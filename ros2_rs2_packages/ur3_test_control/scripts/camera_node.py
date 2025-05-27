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
    
        self.player_publish = self.create_publisher(String, '/player_move', 10)
        self.speaker_publish = self.create_publisher(String, '/TTS', 10)
        self.get_logger().info("Player move publisher created")
        self.image_action_subscription = self.create_subscription(Bool, '/take_image', self.img_action, 10)

        self.bridge = CvBridge()
        self.latest_image = None
        self.get_logger().info('Node started. Displaying image stream. Press Ctrl+C to stop.')
        self.board = [['-'] * 8 for _ in range(8)]
        self.castled = False
        self.get_logger().info('Retrieving the classifier models.')
        self.model = get_model(model_id="reddotdetection/1")
        self.get_logger().info('Got red dot detection.')
        self.BWmodel = get_model(model_id="blackwhite-fkduo/1")
        self.get_logger().info('Got black white detection.')
        self.cell_images = np.empty((8, 8), dtype=object)
        self.takeImg = False
        self.get_logger().info('Ready to play !!!!!!!!!.')

        self.speaker_publish.publish(String(data='Successfully retrieved classifier'))

    def img_action(self, msg):
        self.takeImg = msg.data

    def chess_callback(self, msg):
        self.get_logger().info(f'Camera node got new message: {len(msg.data)}')
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
                chessBoardB = self.process_chessboard(self.latest_image)
                chessStr = np.array2string(chessBoardB, separator=', ')
                self.get_logger().info(f'Occupancy grid from black perspective:\n{chessStr}')

                # Convert to the white perspective to match with the chess node
                chessBoardW = [row[::-1] for row in chessBoardB[::-1]]
                self.cell_images = [row[::-1] for row in self.cell_images[::-1]]

                castlingCheck = False

                # Check if any castling was made
                if chessBoardW[7][0] == 1 and self.board[7][0] == 'R' and chessBoardW[7][4] == 1 and self.board[7][4] == 'K' \
                    and chessBoardW[7][2] == 0 and self.board[7][2] == '-' and chessBoardW[7][3] == 0 and self.board[7][3] == '-' and not self.castled:

                    self.get_logger().info("White Queen-Side Castle")
                    self.speaker_publish.publish(String(data='White Queen Side Castle'))
                    playerMove = [[7, 4], [7, 2], [7, 0], [7, 3]]
                    castlingCheck = True
                    self.castled = True

                if chessBoardW[7][7] == 1 and self.board[7][7] == 'R' and chessBoardW[7][4] == 1 and self.board[7][4] == 'K' \
                    and chessBoardW[7][5] == 0 and self.board[7][5] == '-' and chessBoardW[7][6] == 0 and self.board[7][6] == '-' and not self.castled:
                    self.speaker_publish.publish(String(data='White King Side Castle'))
                    self.get_logger().info("White King-Side Castle")
                    playerMove = [[7, 4], [7, 6], [7, 7], [7, 5]]
                    castlingCheck = True
                    self.castled = True

                #Check for En Passant, no legality check yet
                square_change = 0
                startEnPassant = None
                goalEnPassant = None
                capturedPieceEnPassant = None
                change_list = []
                for i in range(8):
                    for y in range(8):
                        if (chessBoardW[i][y] == 1 and self.board[i][y] != '-') or (chessBoardW[i][y] != 1 and self.board[i][y] == '-'):
                            square_change = square_change + 1
                            change_list = change_list + [[i, y]]
                
                #En Passant Detection
                if square_change == 3:
                    for i in range(3):
                        if self.board[change_list[i][0]][change_list[i][1]] == '-':
                            goalEnPassant = change_list[i]
                            capturedPieceEnPassant = [change_list[i][0] + 1, change_list[i][1]]
                            if self.board[change_list[i][0] + 1][change_list[i][1] + 1] == 'P' and chessBoardW[change_list[i][0] + 1][change_list[i][1] + 1] == 1:
                                startEnPassant = [change_list[i][0] + 1, change_list[i][1] + 1]
                                playerMove = [capturedPieceEnPassant, [9, 9], startEnPassant, goalEnPassant]
                                print("White En Passant")
                                break
                            elif self.board[change_list[i][0] + 1][change_list[i][1] - 1] == 'P' and chessBoardW[change_list[i][0] + 1][change_list[i][1] - 1] == 1:
                                startEnPassant = [change_list[i][0] + 1, change_list[i][1] - 1]
                                playerMove = [capturedPieceEnPassant, [9, 9], startEnPassant, goalEnPassant]
                                print("White En Passant")
                                break

                #Normal and Capture move
                if not castlingCheck and square_change < 3:
                    stop = False
                    for i in range(8):
                        for y in range(8):
                            # the piece has moved
                            if chessBoardW[i][y] == 1 and self.board[i][y] != '-':
                                cellToCheck = self.get_possible_moves(i, y)
                                for j in range(len(cellToCheck)):
                                    #black and white classification goes here
                                    if chessBoardW[cellToCheck[j][0]][cellToCheck[j][1]] == 0:
                                        # There used to be a mistake here as we checked the wrong cell, fixed now
                                        if self.isWhite(self.cell_images[cellToCheck[j][0]][cellToCheck[j][1]]):
                                            pieceInitialPos = [i, y]
                                            pieceFinalPos = cellToCheck[j]
                                            # Capture move, add 9 9 for the chessboard node to delete the captured piece
                                            if self.board[cellToCheck[j][0]][cellToCheck[j][1]] != '-':
                                                playerMove = [pieceFinalPos, [9, 9], pieceInitialPos, pieceFinalPos]
                                            else: # This is a normal move
                                                playerMove = [pieceInitialPos, pieceFinalPos]
                                            stop = True
                                            break
                            if stop:
                                break
                        if stop:
                            break
                        
                if playerMove is None:
                    self.speaker_publish.publish(String(data='illegal move detected'))
                    print("Cannot find player move, trying again ...")
                    return

                self.get_logger().info(f'Piece Start Position: {playerMove[0][0]}, {playerMove[0][1]}; Piece Destination: {playerMove[1][0]}, {playerMove[1][1]} ')
                # # Publish the move to the chess node for stockfish
                toChessNode = ''.join(str(num) for sublist in playerMove for num in sublist)
                self.player_publish.publish(String(data=toChessNode))
                self.speaker_publish.publish(String(data='Move detected'))

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

    def isWhite(self, img):
        prediction = self.BWmodel.infer(img)[0]
        id = prediction.predictions[0].class_id
        if id == 2:
            return True
        return False

    def process_chessboard(self, img):
        h, w = img.shape[:2]
        rows, cols = 8, 8
        cell_h = h // rows
        cell_w = w // cols
        occupancy = np.zeros((8, 8))
        for i in range(rows):
            for j in range(cols):
                y1 = i * cell_h
                y2 = (i + 1) * cell_h
                x1 = j * cell_w
                x2 = (j + 1) * cell_w
                cell_img = img[y1:y2, x1:x2]
                self.cell_images[i][j] = cell_img

                if self.isDot(cell_img) == 0:
                    occupancy[i][j] = 1

        return occupancy

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
            # Standard king moves check only, as castling is handled before this function call
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
