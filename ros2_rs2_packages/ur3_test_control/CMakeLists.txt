cmake_minimum_required(VERSION 3.5)
project(ur3_test_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(joint_publisher src/main.cpp)
ament_target_dependencies(joint_publisher rclcpp sensor_msgs)

add_executable(keyboard_publisher src/keyboard_node.cpp)
ament_target_dependencies(keyboard_publisher rclcpp sensor_msgs)

add_executable(trajectory_node src/trajectory_test.cpp)
ament_target_dependencies(trajectory_node rclcpp sensor_msgs trajectory_msgs geometry_msgs tf2_geometry_msgs)

add_executable(moveIt_test_node src/moveItTest.cpp)
ament_target_dependencies(moveIt_test_node rclcpp sensor_msgs geometry_msgs tf2_geometry_msgs moveit_ros_planning_interface)


install(TARGETS joint_publisher keyboard_publisher trajectory_node moveIt_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
