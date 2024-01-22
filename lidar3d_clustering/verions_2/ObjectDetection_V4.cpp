// 3D LiDAR Object Detection & Tracking using Euclidean Clustering, RANSAC, & Hungarian Algorithm
// RORS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


using namespace std;

#include "obstacle_detector.hpp"

struct BBox
{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    double r = 1.0;
    double g = 1.0;
    double b = 0.0;
};


struct Point {
    float x, y;
};


struct Zone {
    std::vector<Point> corners; // Four corners of the zone

    Zone(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
        corners.push_back(p1);
        corners.push_back(p2);
        corners.push_back(p3);
        corners.push_back(p4);
    }

    bool contains(const pcl::PointXYZ& point) const {
        int count = 0;
        size_t n = corners.size();

        for (size_t i = 0; i < n; i++) {
            size_t j = (i + 1) % n;
            if (isIntersecting(point, corners[i], corners[j])) {
                count++;
            }
        }

        return count % 2 == 1; // Inside if odd number of intersections
    }

private:
    bool isIntersecting(const pcl::PointXYZ& p_orig, const Point& p1, const Point& p2) const {
        Point p = {p_orig.x, p_orig.y};

        if (p.y == p1.y || p.y == p2.y) {
            p.y += 0.0001; 
        }

        if (p.y < std::min(p1.y, p2.y) || p.y > std::max(p1.y, p2.y)) {
            return false;
        }

        if (p.x > std::max(p1.x, p2.x)) {
            return true;
        }

        if (p.x < std::min(p1.x, p2.x)) {
            return false;
        }

        double slope = (p2.y - p1.y) / (p2.x - p1.x);
        double y_intercept = p1.y - slope * p1.x;
        double intersect_x = (p.y - y_intercept) / slope;

        return p.x < intersect_x;
    }
};


class ObjectDetection: public rclcpp::Node
{
private:
    // variables
    float GROUND_THRESHOLD;
    float CLUSTER_THRESH;
    int CLUSTER_MAX_SIZE;
    int CLUSTER_MIN_SIZE;
    size_t obstacle_id_;

    bool USE_PCA_BOX;
    float DISPLACEMENT_THRESH;
    float IOU_THRESH;
    bool USE_TRACKING;
    std::vector<Box> curr_boxes_; 
    std::vector<Box> prev_boxes_;

    Zone front_zone;
    Zone front_zone_warning;



    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;


    void distance_detector(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters);

    void box3dcreation(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header);

    void publisherboxes(std::vector<BBox>&& bboxes, const std_msgs::msg::Header& header);

    void warnning_display(const int warning_code);

    void convex_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters);

    void check_zones(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters);

    void publish_zone(const Zone& zone1, const Zone& zone2);

    visualization_msgs::msg::Marker create_zone_marker(const Zone& zone, int id, const std::string& color);



    // Point Cloud callback
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Subscriber & Publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_seg_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_next;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_warning;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hull_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_publisher_;




public:
    ObjectDetection(/* args */);
    ~ObjectDetection();
};
// to left or right, to the top or down
ObjectDetection::ObjectDetection(/* args */) : Node("lidar3d_clustering_node"), front_zone(Point{-1.2, 0.0}, Point{-1.2, 2.0}, Point{1.2, 2.0}, Point{1.2, 0.0}), front_zone_warning(Point{-1.2, 2.1}, Point{-1.2, 4.0}, Point{1.2, 4.0}, Point{1.2, 2.1}) 
{
    // Parameters
    this->declare_parameter("GROUND_THRESHOLD", 0.2);
    this->declare_parameter("CLUSTER_THRESH", 0.5);
    this->declare_parameter("CLUSTER_MAX_SIZE", 3000);
    this->declare_parameter("CLUSTER_MIN_SIZE", 10);
    this->declare_parameter("USE_PCA_BOX", true);
    this->declare_parameter("DISPLACEMENT_THRESH", 0.2);
    this->declare_parameter("IOU_THRESH", 0.5);
    this->declare_parameter("USE_TRACKING", true);
    

    // Get parameters
    this->get_parameter("GROUND_THRESHOLD", GROUND_THRESHOLD);
    this->get_parameter("CLUSTER_THRESH", CLUSTER_THRESH);
    this->get_parameter("CLUSTER_MAX_SIZE", CLUSTER_MAX_SIZE);
    this->get_parameter("CLUSTER_MIN_SIZE", CLUSTER_MIN_SIZE);
    this->get_parameter("USE_PCA_BOX", USE_PCA_BOX);
    this->get_parameter("DISPLACEMENT_THRESH", DISPLACEMENT_THRESH);
    this->get_parameter("IOU_THRESH", IOU_THRESH);
    this->get_parameter("USE_TRACKING", USE_TRACKING);

    

    
    // Create subscriber
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_roi", 10, std::bind(&ObjectDetection::pointCloudCallback, this, std::placeholders::_1)); // roi points cloud

    // Create publisher
    // ground_seg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10); // ground points
    marker_pub_next = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array_next", 10); // detector objects
    wall_warning = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    // Create point processor
    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();

    hull_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "convex_hull_marker", 10); // Change "convex_hull_marker" and 10 to your desired topic and queue size

    zone_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_zone_array", 10);



    obstacle_id_ = 0;
    // front_zone = Zone(Point{0.0, -1.0}, Point{0.0, 1.0}, Point{2.0, 1.0}, Point{2.0, -1.0});
    // front_zone_warning = Zone(Point{2.0, -1.0}, Point{2.0, 1.0}, Point{4.0, 1.0}, Point{4.0, -1.0});

    RCLCPP_INFO(this->get_logger(), "lidar3d_Clustering_node initialized");

    // Print parameters
    // RCLCPP_INFO(this->get_logger(), "GROUND_THRESHOLD: %f", GROUND_THRESHOLD);
    // RCLCPP_INFO(this->get_logger(), "CLUSTER_THRESH: %f", CLUSTER_THRESH);
    // RCLCPP_INFO(this->get_logger(), "CLUSTER_MAX_SIZE: %d", CLUSTER_MAX_SIZE);
    // RCLCPP_INFO(this->get_logger(), "CLUSTER_MIN_SIZE: %d", CLUSTER_MIN_SIZE);
    // RCLCPP_INFO(this->get_logger(), "USE_PCA_BOX: %d", USE_PCA_BOX);
    // RCLCPP_INFO(this->get_logger(), "DISPLACEMENT_THRESH: %f", DISPLACEMENT_THRESH);
    // RCLCPP_INFO(this->get_logger(), "IOU_THRESH: %f", IOU_THRESH);
    // RCLCPP_INFO(this->get_logger(), "USE_TRACKING: %d", USE_TRACKING);


}

ObjectDetection::~ObjectDetection()
{
}

// Point Cloud callback
void ObjectDetection::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // TODO: Implement a handler for the point cloud callback to avoid empty point cloud

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    // Pass a const pointer (ConstPtr) to segmentPlane.
    auto segmented_clouds = obstacle_detector->segmentPlane(input_cloud, 100, GROUND_THRESHOLD);
    auto cloud_clusters = obstacle_detector->clustering(input_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);



    box3dcreation(std::move(cloud_clusters), msg->header);

    // distance_detector(std::move(cloud_clusters));

    
    convex_hull(std::move(cloud_clusters));

    check_zones(std::move(cloud_clusters));


    // std::cout << "Number of clustersasdasdx223232: " << cloud_clusters.size() << std::endl;

    publish_zone(front_zone,front_zone_warning);


    // Publish ground points
    // sensor_msgs::msg::PointCloud2 ground_cloud_msg;
    // pcl::toROSMsg(*segmented_clouds.first, ground_cloud_msg);
    // ground_cloud_msg.header = msg->header;
    // ground_seg_pub_->publish(ground_cloud_msg);
    
}


void ObjectDetection::box3dcreation(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header)
{

    std::vector<BBox> bboxes;
    
    int num_reasonable_clusters = 0;
    
    for (auto& cluster : cloud_clusters)
    {

        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D<pcl::PointXYZ>(*cluster, min_pt, max_pt);

        BBox bbox;
        bbox.x_min = min_pt[0];
        bbox.y_min = min_pt[1];
        bbox.z_min = min_pt[2];
        bbox.x_max = max_pt[0];
        bbox.y_max = max_pt[1];
        bbox.z_max = max_pt[2];

        bboxes.push_back(bbox);
        num_reasonable_clusters++;

    }

    publisherboxes(std::move(bboxes), header);

}

void ObjectDetection::publisherboxes(std::vector<BBox>&& bboxes, const std_msgs::msg::Header& header){
//==================================== Drawing Boxes  ====================================

    visualization_msgs::msg::MarkerArray marker_array;


    int id = 0;
    const std_msgs::msg::Header& inp_header = header;
    // Create a marker for each bounding box
    for (const auto& bbox : bboxes)
    {
        // Create the marker for the top square
        visualization_msgs::msg::Marker top_square_marker;
        top_square_marker.header = inp_header;
        top_square_marker.ns = "bounding_boxes";
        top_square_marker.id = id++;
        top_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        top_square_marker.action = visualization_msgs::msg::Marker::ADD;
        top_square_marker.pose.orientation.w = 1.0;
        top_square_marker.scale.x = 0.06;
        top_square_marker.color.r = bbox.r;
        top_square_marker.color.g = bbox.g;
        top_square_marker.color.b = bbox.b;
        top_square_marker.color.a = 1.0;

        // Add the points to the top square marker
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
        p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
        p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
        p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;
        top_square_marker.points.push_back(p1);
        top_square_marker.points.push_back(p2);
        top_square_marker.points.push_back(p3);
        top_square_marker.points.push_back(p4);
        top_square_marker.points.push_back(p1);

        // Add the top square marker to the array
        marker_array.markers.push_back(top_square_marker);

        // Create the marker for the bottom square
        visualization_msgs::msg::Marker bottom_square_marker;
        bottom_square_marker.header = inp_header;
        bottom_square_marker.ns = "bounding_boxes";
        bottom_square_marker.id = id++;
        bottom_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        bottom_square_marker.action = visualization_msgs::msg::Marker::ADD;
        bottom_square_marker.pose.orientation.w = 1.0;
        bottom_square_marker.scale.x = 0.04;
        bottom_square_marker.color.r = bbox.r;
        bottom_square_marker.color.g = bbox.g;
        bottom_square_marker.color.b = bbox.b;
        bottom_square_marker.color.a = 1.0;

        // Add the points to the bottom square marker
        geometry_msgs::msg::Point p5, p6, p7, p8;
        p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
        p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
        p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
        p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;

        bottom_square_marker.points.push_back(p5);
        bottom_square_marker.points.push_back(p6);
        bottom_square_marker.points.push_back(p7);
        bottom_square_marker.points.push_back(p8);
        bottom_square_marker.points.push_back(p5); // connect the last point to the first point to close the square

        // Add the bottom square marker to the marker array
        marker_array.markers.push_back(bottom_square_marker);


        // Create the marker for the lines connecting the top and bottom squares
        visualization_msgs::msg::Marker connecting_lines_marker;
        connecting_lines_marker.header = inp_header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id++;
        connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        connecting_lines_marker.pose.orientation.w = 1.0;
        connecting_lines_marker.scale.x = 0.04;
        connecting_lines_marker.color.r = 1.0;
        connecting_lines_marker.color.g = 0.0;
        connecting_lines_marker.color.b = 0.0;
        connecting_lines_marker.color.a = 1.0;

        // Add the points to the connecting lines marker
        connecting_lines_marker.points.push_back(p1);
        connecting_lines_marker.points.push_back(p5);

        connecting_lines_marker.points.push_back(p2);
        connecting_lines_marker.points.push_back(p6);

        connecting_lines_marker.points.push_back(p3);
        connecting_lines_marker.points.push_back(p7);

        connecting_lines_marker.points.push_back(p4);
        connecting_lines_marker.points.push_back(p8);

        // Add the connecting lines marker to the marker array
        marker_array.markers.push_back(connecting_lines_marker);


        // Create a marker for the corners
        visualization_msgs::msg::Marker corner_marker;
        corner_marker.header = inp_header;
        corner_marker.ns = "bounding_boxes";
        corner_marker.id = id++;
        corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
        corner_marker.action = visualization_msgs::msg::Marker::ADD;
        corner_marker.pose.orientation.w = 1.0;
        corner_marker.scale.x = 0.2;
        corner_marker.scale.y = 0.2;
        corner_marker.scale.z = 0.2;
        corner_marker.color.r = bbox.r;
        corner_marker.color.g = 0.2;
        corner_marker.color.b = 0.5;
        corner_marker.color.a = 0.64;

        // Create a sphere for each corner and add it to the marker array

        corner_marker.pose.position = p1;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p2;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p3;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p4;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p5;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p6;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p7;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p8;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);


        rclcpp::Duration marker_lifetime = rclcpp::Duration::from_seconds(3);


        top_square_marker.lifetime = marker_lifetime;
        bottom_square_marker.lifetime = marker_lifetime;
        connecting_lines_marker.lifetime = marker_lifetime;
        corner_marker.lifetime = marker_lifetime;

        marker_pub_next->publish(marker_array);
    }



}


void ObjectDetection::distance_detector(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters)
{

    float thresh_dist = 1.0; // Threshold for very close obstacles
    float thresh_dist_2 = 2.7; // Threshold for moderately close obstacles
    float thresh_dist_3 = 7.0; // Threshold for distant obstacles
    int highest_warning_code = 4; // To store the highest severity warning code


    for (auto& cluster : cloud_clusters)
    {
        float min_distance = std::numeric_limits<float>::max();
        int warning_code = 4;

        for (const auto& point : cluster->points) {
            float distance = sqrt(point.x * point.x + point.y * point.y);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }

        if (min_distance < thresh_dist) {
            warning_code = 1; // Very close obstacle
        } else if (min_distance >= thresh_dist && min_distance < thresh_dist_2) {
            warning_code = 2; // Moderately close obstacle
        } else if (min_distance >= thresh_dist_2 && min_distance < thresh_dist_3) {
            warning_code = 3; // Distant obstacle
        }

        if (warning_code < highest_warning_code) {
            highest_warning_code = warning_code;
        }
        

        std::cout << " Warning code: " << warning_code << " Distance: " << min_distance << std::endl;

    }

    if (highest_warning_code == 1) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 220] Obstacle detected in less than 1.0m radius");
    } else if (highest_warning_code == 2) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 330] Obstacle detected in 1.0 to 4.5m radius");
    } else if (highest_warning_code == 3) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 550] Obstacle detected in 4.5 to 7.0m radius");
    } else {
        RCLCPP_INFO(this->get_logger(), "[INFO 000] No obstacle detected");
    }

    warnning_display(highest_warning_code);

}



void ObjectDetection::warnning_display(const int warning_code)
{

    visualization_msgs::msg::Marker wall_marker;
    wall_marker.header.frame_id = "detector_wall";
    wall_marker.header.stamp = rclcpp::Clock().now();
    wall_marker.ns = "wall";
    wall_marker.id = 0;
    wall_marker.type = visualization_msgs::msg::Marker::CUBE;
    wall_marker.action = visualization_msgs::msg::Marker::ADD;
    wall_marker.pose.position.x = 0.0;
    wall_marker.pose.position.y = 0.0;
    wall_marker.pose.position.z = 0.0;
    wall_marker.pose.orientation.x = 0.0;
    wall_marker.pose.orientation.y = 0.0;
    wall_marker.pose.orientation.z = 0.0;
    wall_marker.pose.orientation.w = 1.0;
    wall_marker.scale.x = 2.5;
    wall_marker.scale.y = 0.05;
    wall_marker.scale.z = 1.0;
    wall_marker.color.a = 0.35;

    if (warning_code == 1) {
        wall_marker.color.r = 1.0;
        wall_marker.color.g = 0.0;
        wall_marker.color.b = 0.0;
    } else if (warning_code == 2) {
        wall_marker.color.r = 1.0;
        wall_marker.color.g = 1.0;
        wall_marker.color.b = 0.0;
    } else if (warning_code == 3) {
        wall_marker.color.r = 0.0;
        wall_marker.color.g = 1.0;
        wall_marker.color.b = 0.0;
    } else {
        wall_marker.color.r = 0.0;
        wall_marker.color.g = 1.0;
        wall_marker.color.b = 0.0;
    }

    wall_warning->publish(wall_marker);

}

void ObjectDetection::convex_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters)
{
    if (!cloud_clusters.empty())
    {
        int index = 0;  // Declare an index variable
        for (auto& cluster : cloud_clusters)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConvexHull<pcl::PointXYZ> hull;
            hull.setInputCloud(cluster); // Pass each individual cluster
            hull.setDimension(2);
            hull.reconstruct(*convexHull);

            if (convexHull->empty())
            {
                RCLCPP_INFO(this->get_logger(), "Convex hull is empty.");
                continue;
            }

            if (hull.getDimension() == 2)
            {
                std::vector<geometry_msgs::msg::Point> hull_points;
                for (const auto& point : convexHull->points)
                {
                    geometry_msgs::msg::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = 0.0; // Since we are working in 2D
                    hull_points.push_back(p);
                }
                // Close the loop
                hull_points.push_back(hull_points.front());

                visualization_msgs::msg::Marker hull_marker;
                hull_marker.header.frame_id = "base_footprint";  // Replace with your frame ID
                hull_marker.header.stamp = this->get_clock()->now();
                hull_marker.ns = "hull";
                hull_marker.id = index;  // Use the index variable
                hull_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                hull_marker.action = visualization_msgs::msg::Marker::ADD;
                hull_marker.scale.x = 0.1;
                hull_marker.color.r = 0.0;
                hull_marker.color.g = 0.0;
                hull_marker.color.b = 1.0;
                hull_marker.color.a = 1.0;
                hull_marker.points = hull_points;

                hull_publisher_->publish(hull_marker);

                // RCLCPP_INFO(this->get_logger(), "Hull size: %zu", convexHull->size());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "The chosen hull dimension is not correct.");
            }

            index++;  // Increment the index variable
        }
    }
}

void ObjectDetection::check_zones(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters) 
{

    int highest_warning_code = 4; // To store the highest severity warning code

    for (const auto& cluster : cloud_clusters) 
    {

        int warning_code = 4;
        if (cluster->empty()) continue;

        // Initialize the closest point to a large distance
        pcl::PointXYZ closest_point;
        float min_distance = std::numeric_limits<float>::max();

        // Find the closest point in the cluster
        for (const auto& point : cluster->points) {
            float distance = sqrt(point.x * point.x + point.y * point.y); // Assuming vehicle is at (0,0)
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = point;
            }
        }


        if (front_zone.contains(closest_point)) {
            // RCLCPP_INFO(this->get_logger(), "[WARNING 220] at distance: %f", min_distance);
            warning_code = 1;
        } else if(front_zone_warning.contains(closest_point)){
            // RCLCPP_INFO(this->get_logger(), "[WARNING 330] at distance: %f", min_distance);
            warning_code = 2;
        }

        if (warning_code < highest_warning_code) {
            highest_warning_code = warning_code;
        }

    }

    if (highest_warning_code == 1) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 220] Obstacle detected in red zone");
    } else if (highest_warning_code == 2) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 330] Obstacle detected in yellow zone");
    } else {
        RCLCPP_INFO(this->get_logger(), "[INFO 000] No obstacle detected");
    }

    warnning_display(highest_warning_code);

}


void ObjectDetection::publish_zone(const Zone& zone1, const Zone& zone2) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Create and add markers for each zone
    visualization_msgs::msg::Marker marker1 = create_zone_marker(zone1, 1000, "red");
    visualization_msgs::msg::Marker marker2 = create_zone_marker(zone2, 1001, "yellow");

    marker_array.markers.push_back(marker1);
    marker_array.markers.push_back(marker2);

    // Publish the marker array
    zone_publisher_->publish(marker_array);
}

visualization_msgs::msg::Marker ObjectDetection::create_zone_marker(const Zone& zone, int id, const std::string& color) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link"; // Or your relevant frame
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "zone";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // Line width
    marker.color.a = 1.0; // Alpha (opacity)

    if (color == "yellow") {
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; // Yellow
    } else {
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Red (default)
    }

    for (const auto& corner : zone.corners) {
        geometry_msgs::msg::Point p;
        p.x = corner.x; p.y = corner.y; p.z = 0;
        marker.points.push_back(p);
    }
    marker.points.push_back(marker.points.front()); // Close the loop

    return marker;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

