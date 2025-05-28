#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo:/usr/share/gazebo-11
export LIBGL_ALWAYS_SOFTWARE=true
export ROBOFLOW_API_KEY=4iX1AGkQPTpy0JuUvYAL
export ROS_DOMAIN_ID=74
source "$SCRIPT_DIR/../../install/setup.bash"

# Run the ros2 launch command
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1280x720x30 \
  enable_depth:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  unite_imu_method:=0 \
  enable_sync:=false \
  timestamp_domain:=1 \
  global_time_enable:=false \
  color_qos:=SENSOR_DATA \
  rgb_camera.color_format:=RGB8 \
  log_level:=WARN &
  
python3 "$SCRIPT_DIR/python_tests/arucoChecker.py" &

ros2 run ur3_test_control camera_node.py &

ros2 launch ur3_test_control moveL.launch.py &

wait
