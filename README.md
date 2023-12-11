# lidar_sdv
 lidar algorithms to detect and stop the car 
 

source /opt/ros/humble/setup.bash #for ro2 humble
source install/setup.bash

colcon build --symlink-install

ros2 launch sdv_robot_description display.launch.py

ros2 bag play /home/genis/ros_bags/vanttec2/rosbag2_2023_03_23-23_34_21_0.db3

ros2 bag play /home/genis/ros_bags/vantec//rosbag2_2023_05_23-18_50_43_0.db3

ros2 launch sdv_lidar_processing lidar_Processing.launch.py

