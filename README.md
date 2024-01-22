# lidar_sdv
 lidar algorithms to detect and stop the car 
 

## Install To ise this repo and the lidar functions
```bash
sudo apt-get install libeigen3-dev
sudo apt install libpcl-dev
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-pcl-ros
sudo apt install ros-humble-vision-msgs
```

 ## Set up ROS2
```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
source install/setup.bash
```

# ROS BAGs EXAMPLES
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

# Lidar 3D clustering final version
```bash
colcon build --packages-select lidar3d_clustering
source install/setup.bash
ros2 launch lidar3d_clustering lidar3d.launch.py
```
 
# Optimus variables 
 ```bash
lidar3d_Clustering_node:
  ros__parameters:
    GROUND_THRESHOLD: 0.03
    CLUSTER_THRESH: 0.18
    CLUSTER_MAX_SIZE: 5000
    CLUSTER_MIN_SIZE: 10
    USE_PCA_BOX: 0
    DISPLACEMENT_THRESH: 1.0
    IOU_THRESH: 1.1
    USE_TRACKING: 1

```

# In the lab computer
 ```bash
ros2 bag play /home/rola/ros2/ros_bags/vanttec_2/rosbag2_2023_05_23-18_50_43_0-001.db3
```
 

