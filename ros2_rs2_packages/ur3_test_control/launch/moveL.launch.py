import launch
import os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

#for launch file
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "visual_parameters.yaml"]
    )
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " ",
        "robot_ip:=192.168.0.250",  # Match your robot_ip from ur_control.launch.py
        " ",
        "joint_limit_params:=",
        joint_limit_params,
        " ",
        "kinematics_params:=",
        kinematics_params,
        " ",
        "physical_params:=",
        physical_params,
        " ",
        "visual_params:=",
        visual_params,
        " ",
        "safety_limits:=",
        "true",
        " ",
        "safety_pos_margin:=",
        "0.15",
        " ",
        "safety_k_position:=",
        "20",
        " ",
        "name:=",
        "ur",
        " ",
        "ur_type:=",
        "ur3e",
        " ",
        "prefix:=",
        '""',
        " ",])

    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " ",
        "name:=",
        "ur",
        " ",
        "prefix:=",
        '""',
        " ",])

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic

def generate_launch_description():
    # Paths to configuration files
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "config",
        "kinematics.yaml"
    ])
    
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    
    # Your test node
    demo_node = Node(
        package="ur3_test_control",
        executable="moveIt_test_node",
        name="moveIt_test",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    chess_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ur3_test_control",
                executable="chess_node",
                name="Chess_Board",
                output="screen",
            )
        ]
    )

    cam_node = Node(
        package="ur3_test_control",
        executable="camera_node.py",
        name="Camera",
        output="screen",
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_moveit_config'), 'launch', 'ur_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            # 'launch_rviz': 'false'
        }.items()
    )
    
    return launch.LaunchDescription([ur_moveit_launch, demo_node, cam_node, chess_node])