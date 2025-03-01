Using AI and UR3e Cobot to play different variances of chess 
To run the simulation, first run : 

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 use_fake_hardware:=true launch_rviz:=true

then: ros2 launch ur3_test_control moveL.launch.py

