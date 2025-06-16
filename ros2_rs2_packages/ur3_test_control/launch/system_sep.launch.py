import launch
import os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

# Add environment variable to control logging
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'
os.environ['RCUTILS_LOGGING_BUFFERED_STREAM'] = '1'
os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '1'

# Set global logging level to ERROR
os.environ['RCUTILS_LOGGING_MIN_SEVERITY'] = 'ERROR'

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
        "robot_ip:=192.168.0.250",
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
        ])

    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
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
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    
    # Get the package share directory
    pkg_share = get_package_share_directory('ur3_test_control')
    
    main_node = Node(
        package="ur3_test_control",
        executable="moveIt_test_node",
        name="moveIt_test",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"simulation_mode": False},
        ],
        arguments=['--ros-args', '--log-level', 'moveIt_test:=INFO']
    )

    chess_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ur3_test_control",
                executable="chess_node",
                name="chess_node",
                output="screen",
            )
        ]
    )

    aruco_node = Node(
        package="ur3_test_control",
        executable=os.path.join(pkg_share, '..', '..', 'src', 'ur3_test_control', 'scripts', 'arucoChecker.py'),
        name="aruco_checker",
        output="screen",
    )

    camera_node = Node(
        package="ur3_test_control",
        executable="camera_node.py",
        name="camera_node",
        output="screen",
    )
    
    return launch.LaunchDescription([
        main_node,
        chess_node,
        aruco_node,
        camera_node,
    ]) 