#!/usr/bin/env python3

import subprocess
import os

# Configuration variables - these can be modified as needed
A1_X_POSITION = 0.1316  # X-coordinate of the a1 square in meters
A1_Y_POSITION = 0.4674  # Y-coordinate of the a1 square in meters
SQUARE_SIZE = 35/1000  # Size of each square in meters (35mm)
Z_HEIGHT = 0.026  # Height from the ground for all pieces

def spawn_piece(piece_type, position):
    """
    Spawn a chess piece in Gazebo
    
    Args:
        piece_type (str): Type of the piece (e.g., 'black_pawn', 'white_queen')
        position (str): Chess position (e.g., 'a1', 'e4')
    """
    # Extract position
    col = position[0].lower()
    row = position[1]
    
    # Convert chess notation to array indices
    col_idx = ord(col) - ord('a')  # a=0, b=1, c=2, etc.
    row_idx = int(row) - 1        # 1=0, 2=1, 3=2, etc.
    
    # Calculate coordinates from a1 position
    x = A1_X_POSITION - (col_idx * SQUARE_SIZE)
    y = A1_Y_POSITION - (row_idx * SQUARE_SIZE)
    
    # Create entity name
    entity_name = f"{piece_type}_{position}"
    
    # Full path to the model SDF file
    model_file = os.path.expanduser(f"~/git/RS2/gazebo_models/{piece_type}/model.sdf")
    
    # Build the command
    cmd = [
        "ros2", "run", "gazebo_ros", "spawn_entity.py",
        "-file", model_file,
        "-entity", entity_name,
        "-x", str(x),
        "-y", str(y),
        "-z", str(Z_HEIGHT),
        "-robot_namespace", entity_name
    ]
    
    # Execute the command
    print(f"Spawning {piece_type} at position {position} (x={x:.6f}, y={y:.6f}, z={Z_HEIGHT})")
    subprocess.run(cmd)
    print(f"Spawned {entity_name}")

def setup_initial_board():
    """Set up a complete chess board with all pieces in their initial positions"""
    
    # Place pawns
    for col in "abcdefgh":
        spawn_piece("black_pawn", f"{col}7")
        spawn_piece("white_pawn", f"{col}2")
    
    # Place rooks
    spawn_piece("black_rook", "a8")
    spawn_piece("black_rook", "h8")
    spawn_piece("white_rook", "a1")
    spawn_piece("white_rook", "h1")
    
    # Place knights
    spawn_piece("black_knight", "b8")
    spawn_piece("black_knight", "g8")
    spawn_piece("white_knight", "b1")
    spawn_piece("white_knight", "g1")
    
    # Place bishops
    spawn_piece("black_bishop", "c8")
    spawn_piece("black_bishop", "f8")
    spawn_piece("white_bishop", "c1")
    spawn_piece("white_bishop", "f1")
    
    # Place queens
    spawn_piece("black_queen", "d8")
    spawn_piece("white_queen", "d1")
    
    # Place kings
    spawn_piece("black_king", "e8")
    spawn_piece("white_king", "e1")

def spawn_custom_piece():
    """Interactive function to spawn a single piece at a specified position"""
    print("Available pieces:")
    pieces = [
        "black_pawn", "black_rook", "black_bishop", "black_knight", 
        "black_king", "black_queen", "white_pawn", "white_rook", 
        "white_knight", "white_bishop", "white_king", "white_queen"
    ]
    
    for i, piece in enumerate(pieces, 1):
        print(f"{i}. {piece}")
    
    try:
        piece_idx = int(input("Enter piece number: ")) - 1
        if piece_idx < 0 or piece_idx >= len(pieces):
            print("Invalid piece number")
            return
        
        piece_type = pieces[piece_idx]
        position = input("Enter position (e.g., e4): ")
        
        if len(position) != 2 or position[0] not in "abcdefgh" or position[1] not in "12345678":
            print("Invalid position. Format must be like 'a1', 'e4', etc.")
            return
            
        spawn_piece(piece_type, position)
        
    except ValueError:
        print("Please enter a valid number")

def modify_board_parameters():
    """Modify the board starting position and square size"""
    global A1_X_POSITION, A1_Y_POSITION, SQUARE_SIZE, Z_HEIGHT
    
    print(f"Current parameters:")
    print(f"A1 position: X={A1_X_POSITION}, Y={A1_Y_POSITION}")
    print(f"Square size: {SQUARE_SIZE} meters ({SQUARE_SIZE*1000} mm)")
    print(f"Piece height: {Z_HEIGHT} meters")
    
    try:
        A1_X_POSITION = float(input("Enter A1 X position (meters): "))
        A1_Y_POSITION = float(input("Enter A1 Y position (meters): "))
        square_mm = float(input("Enter square size (mm): "))
        SQUARE_SIZE = square_mm / 1000
        Z_HEIGHT = float(input("Enter piece height (meters): "))
        
        print("Parameters updated successfully")
    except ValueError:
        print("Please enter valid numbers")

if __name__ == "__main__":
    print("Chess Piece Spawner for ROS2 Gazebo")
    print("1. Spawn complete initial chess set")
    print("2. Spawn a single piece at a specific position")
    print("3. Modify board parameters")
    
    choice = input("Enter your choice (1, 2, or 3): ")
    
    if choice == "1":
        setup_initial_board()
    elif choice == "2":
        spawn_custom_piece()
    elif choice == "3":
        modify_board_parameters()
        print("1. Spawn complete initial chess set")
        print("2. Spawn a single piece at a specific position")
        next_choice = input("Enter your next choice (1 or 2): ")
        if next_choice == "1":
            setup_initial_board()
        elif next_choice == "2":
            spawn_custom_piece()
        else:
            print("Invalid choice")
    else:
        print("Invalid choice")
