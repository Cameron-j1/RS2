import launch
import os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

#for launch file
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

# Add environment variable to control logging
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'
os.environ['RCUTILS_LOGGING_BUFFERED_STREAM'] = '1'
os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '1'

# Set global logging level to ERROR
os.environ['RCUTILS_LOGGING_MIN_SEVERITY'] = 'ERROR'

# Specifically suppress MoveGroup and related loggers
os.environ['RCUTILS_LOGGING_MIN_SEVERITY_MOVEIT'] = 'ERROR'
os.environ['RCUTILS_LOGGING_MIN_SEVERITY_UR_MOVEIT'] = 'ERROR'
os.environ['RCUTILS_LOGGING_MIN_SEVERITY_MOVEIT_MOVE_GROUP'] = 'ERROR'
os.environ['RCUTILS_LOGGING_MIN_SEVERITY_MOVEIT_ROS'] = 'ERROR'
os.environ['RCUTILS_LOGGING_MIN_SEVERITY_MOVEIT_PLUGINS'] = 'ERROR'
os.environ['RCUTILS_LOGGING_MIN_SEVERITY_MOVEIT_SIMPLE_CONTROLLER_MANAGER'] = 'ERROR'
os.environ['RCUTILS_LOGGING_MIN_SEVERITY_MOVEIT_TRAJECTORY_EXECUTION_MANAGER'] = 'ERROR'

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
        ])

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
        # Override the global logging level for this specific node
        arguments=['--ros-args', '--log-level', 'moveIt_test:=INFO']
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
        executable="image_processing_exe",
        name="image_processing",
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
    
    return launch.LaunchDescription([
        ur_moveit_launch,
        main_node,
        chess_node,
        #cam_node
    ])