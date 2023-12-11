#include <rclcpp/rclcpp.hpp>
// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// C++
#include <iostream>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>

// OpenCV and ROS
#include <image_geometry/pinhole_camera_model.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;


class Processing_lidar : public rclcpp::Node
{
private:

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;

public:
    Processing_lidar(/* args */);
    ~Processing_lidar();
};

Processing_lidar::Processing_lidar(/* args */) : Node("Lidar_Processing_node")

{
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 10, std::bind(&Processing_lidar::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Lidar_Processing_node initialized");

}

Processing_lidar::~Processing_lidar()
{
}

void Processing_lidar::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "ObstacleDectector_node initialized");


}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Processing_lidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
