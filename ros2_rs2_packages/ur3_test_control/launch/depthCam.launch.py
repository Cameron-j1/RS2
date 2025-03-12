import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ur3_test_control')
    world_file = os.path.join(pkg_dir, 'world', 'cubeWorld.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'd435i_robot.urdf')

    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'
        # ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': open(urdf_file).read()}]
        # ),
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     arguments=['-entity', 'd435i_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0'],
        #     output='screen'
        # ),

        Node(
        package="ur3_test_control",
        executable="camera_node.py",
        name="Camera",
        output="screen",
        ),
    ])