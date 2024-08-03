# Ros2_PathPlanning
Source: AI_winter ros_motion_planning


Clone src về colcon build
param saver dùng để nạp planner và file param ở trong /turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2

Nạp planner plugin xong tiến hành chạy test code với model burger: 
Terminal 1 : ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
Terminal 2 : ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yamlburger
