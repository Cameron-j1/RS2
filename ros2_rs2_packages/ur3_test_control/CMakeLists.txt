cmake_minimum_required(VERSION 3.5)
project(ur3_test_control)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(moveIt_test_node src/moveItTest.cpp)
ament_target_dependencies(moveIt_test_node rclcpp std_msgs sensor_msgs geometry_msgs tf2_geometry_msgs moveit_ros_planning_interface)

add_executable(chess_node src/chessboard.cpp)
ament_target_dependencies(chess_node rclcpp std_msgs ament_index_cpp)

add_executable(image_processing_exe src/image_processing.cpp)
ament_target_dependencies(image_processing_exe rclcpp std_msgs sensor_msgs geometry_msgs tf2_geometry_msgs moveit_ros_planning_interface cv_bridge OpenCV)

target_link_libraries(chess_node
  sfml-graphics
  sfml-window
  sfml-system
  Threads::Threads
)

install(TARGETS moveIt_test_node chess_node image_processing_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY images world urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/camera_node.py
  scripts/img_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
