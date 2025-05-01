cmake_minimum_required(VERSION 3.5)
project(ur3_test_control)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
find_package(rclpy REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(chess_node src/chessboard.cpp)
ament_target_dependencies(chess_node rclcpp std_msgs ament_index_cpp)

target_link_libraries(chess_node
  sfml-graphics
  sfml-window
  sfml-system
  Threads::Threads
)

install(TARGETS chess_node 
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch images
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS scripts/servo_control_node.py DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})


ament_package()
