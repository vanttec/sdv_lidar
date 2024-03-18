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

# 1. Model description
```bash
colcon build --packages-select niagara_model
source install/setup.bash
ros2 launch sdv_robot_description display.launch.py
```

# 2. Ground LIDAR 
```bash
colcon build --packages-select lidar_ground_getter
source install/setup.bash
ros2 launch lidar_ground_getter lidar_ground.launch.py
```

# 3. Voxel and RIO Lidar 3D
```bash
colcon build --packages-select voxel_grid_filter
source install/setup.bash
ros2 launch voxel_grid_filter filter.launch.py
```



# 4. Lidar 3D clusterings
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


# Topics descriptions
|     topics     | description  | Type  | Origin pkg |
| :-------- | :------- | :-------------------------------- |:-------- |
| `/points_roi` | receives the points from the /velodyne points topic and creates a region of interest. | PointCloud2 | voxel_grid_filter |
| `/points_ground` | It receives the points from the region of interest from the /points_roi topic and removes the points from the floor to only leave the objects. | PointCloud2 | lidar_ground_getter |
| `/object_bounding_box` | the bounding box of the object | MarkerArray | lidar3d_clustering | 
| `/warning_visualization_tool` | Warning display tool to visually discover if the object is far away or close. | Marker | lidar3d_clustering | 
| `/visualization_zone` | visualization tool to observe risk areas. | MarkerArray | lidar3d_clustering | 
| `/warning_status` | int topic to warn by numbers if an object is closed or away. | Int32 | lidar3d_clustering | 

# warning_status topic descriptions
|     # Int32    | description  | 
| :-------- | :------- | 
| `1`      | [WARNING 220] Obstacle detected in red zone. |
| `2`      | [WARNING 330] Obstacle detected in yellow zone. | 
| `4`      | [INFO 000] No obstacle detected in any zone. |


# In the lab computer
 ```bash
ros2 bag play /home/rola/ros2/ros_bags/vanttec_2/rosbag2_2023_05_23-18_50_43_0-001.db3
ros2 bag play /home/genis-rog/ros2/ros_bags/new_vanttec/rosbag2_2024_03_08-22_50_21/rosbag2_2024_03_08-22_50_21_0.db3 --topics /velodyne_points
```
 
