# lidar_sdv
 lidar algorithms to detect and stop the car 
 

## Install
```bash
sudo apt-get install libeigen3-dev
```

 ## Set up ROS2
```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
source install/setup.bash
```

# ROS BAGs
```bash
ros2 bag play /home/genis/ros_bags/vanttec2/rosbag2_2023_03_23-23_34_21_0.db3
ros2 bag play /home/genis/ros_bags/vantec//rosbag2_2023_05_23-18_50_43_0.db3
```

# Model description
```bash
colcon build --packages-select niagara_model
source install/setup.bash
ros2 launch sdv_robot_description display.launch.py
```

# Voxel and RIO Lidar 3D
```bash
colcon build --packages-select voxel_grid_filter
source install/setup.bash
ros2 launch voxel_grid_filter filter.launch.py
```

# Lidar 3D processing 
```bash
colcon build --packages-select sdv_lidar_processing
source install/setup.bash
ros2 launch sdv_lidar_processing lidar_Processing.launch.py
```
 
# Lidar 3D clustering 
```bash
colcon build --packages-select lidar3d_clustering
source install/setup.bash
ros2 launch lidar3d_clustering lidar3d.launch.py
```
 
