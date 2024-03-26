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


#include "obstacle_detector.hpp"


class Processing_lidar : public rclcpp::Node
{
private:
    float GROUND_THRESHOLD = 0.3;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;



    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_seg_pub_;

public:
    Processing_lidar(/* args */);
    ~Processing_lidar();
};

Processing_lidar::Processing_lidar(/* args */) : Node("Lidar_Processing_node")

{
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points_roi", 10, std::bind(&Processing_lidar::pointCloudCallback, this, std::placeholders::_1));
    ground_seg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10);

    // Create point processor
    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();

    RCLCPP_INFO(this->get_logger(), "Lidar_Processing_node initialized");
}

Processing_lidar::~Processing_lidar()
{
}

void Processing_lidar::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *input_cloud);


    // Convert your PointXYZI cloud to PointXYZ cloud if necessary.
    pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input_cloud, *converted_cloud);

    // Pass a const pointer (ConstPtr) to segmentPlane.
    auto segmented_clouds = obstacle_detector->segmentPlane(converted_cloud, 30, GROUND_THRESHOLD);


    Processing_lidar::PointCloudMsg downsampled_cloud_msg;
    pcl::toROSMsg(*(segmented_clouds.first), downsampled_cloud_msg);

    ground_seg_pub_->publish(downsampled_cloud_msg);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Processing_lidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
