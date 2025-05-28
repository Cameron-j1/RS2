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
from launch.actions import SetEnvironmentVariable


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
    # Set required environment variables for Gazebo
    # Get the directory where this launch file is located
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    # Navigate from launch/ to the gazebo_models directory: ../../../gazebo_models
    gazebo_models_path = os.path.join(launch_file_dir, '..', '..', '..', 'gazebo_models')
    
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [os.environ.get('GAZEBO_MODEL_PATH', ''), ':',
         os.path.abspath(gazebo_models_path)]
    )
    
    gazebo_model_database_uri = SetEnvironmentVariable(
        'GAZEBO_MODEL_DATABASE_URI',
        ''
    )
    
    # Get the path to our custom world file
    world_file_path = os.path.join(
        get_package_share_directory('ur3_test_control'),
        'world',
        'full_complete_v3.world'
    )
    
    # Paths to configuration files
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "config",
        "kinematics.yaml"
    ])
    
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    
    # Include the UR simulation launch file with our custom world
    ur_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_simulation_gazebo'),
                'launch',
                'ur_sim_moveit.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'start_joint_controller': 'true',
            'launch_rviz': 'false',
            'gazebo_gui': 'true',
            'world': world_file_path
        }.items()
    )
    
    # Add longer delay and simulation time parameter for MoveIt node
    main_node = TimerAction(
        period=10.0,  # Increased delay to ensure Gazebo is fully started
        actions=[
            Node(
                package="ur3_test_control",
                executable="moveIt_test_node",
                name="moveIt_test",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {"simulation_mode": True},
                    {"use_sim_time": True}  # Important: use simulation time
                ],
            )
        ]
    )

    chess_node = TimerAction(
        period=12.0,  # Start after MoveIt node
        actions=[
            Node(
                package="ur3_test_control",
                executable="chess_node",
                name="Chess_Board",
                output="screen",
                parameters=[{"use_sim_time": True}]
            )
        ]
    )

    cam_node = TimerAction(
        period=8.0,  # Start after simulation is stable
        actions=[
            Node(
                package="ur3_test_control",
                executable="image_processing_exe",
                name="image_processing",
                output="screen",
                parameters=[{"use_sim_time": True}]
            )
        ]
    )

    gazebo_manager_node = TimerAction(
        period=8.0,  # Start after simulation is stable
        actions=[
            Node(
                package="ur3_test_control",
                executable="gazebo_manager.py",
                name="gazebo_manager",
                output="screen",
                parameters=[{"use_sim_time": True}]
            )
        ]
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
        gazebo_model_path,
        gazebo_model_database_uri,
        ur_simulation_launch,
        main_node,
        chess_node,
        cam_node,
        gazebo_manager_node
    ])