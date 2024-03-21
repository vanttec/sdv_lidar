
#include <rclcpp/rclcpp.hpp>

// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

// Eigen
#include <Eigen/Dense>

using namespace std;


class LidarImuSync : public rclcpp::Node
{
private:
    /* data */
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloudMsg, PointCloudMsg>;
public:
    LidarImuSync(/* args */);
    ~LidarImuSync();
};

LidarImuSync::LidarImuSync(/* args */) : Node("lidar_imu_sync")
{
}

LidarImuSync::~LidarImuSync()
{
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarImuSync>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}