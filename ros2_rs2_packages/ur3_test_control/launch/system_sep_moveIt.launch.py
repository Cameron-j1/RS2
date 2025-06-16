import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

def generate_launch_description():
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
    ]) 