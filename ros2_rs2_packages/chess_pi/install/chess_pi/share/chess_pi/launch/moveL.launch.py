import launch
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    chess_node = Node(
                package="ur3_test_control",
                executable="chess_node",
                name="Chess_Board",
                output="screen",
            )
    
    return launch.LaunchDescription([chess_node])