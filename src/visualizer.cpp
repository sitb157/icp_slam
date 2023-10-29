#include "icp_slam/visualizer.hpp"

// Initialize
Visualizer::Visualizer(const std::string &node_name) : Node(node_name) {

    // Publisher
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/point_cloud", 10);
    
    // Set parameter for visualizer

    // Massages handler
    msg_handler_ = std::make_shared<MsgHandler>();

}

// Visualize Point Cloud
void Visualizer::visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ> point_cloud, uint8_t r, uint8_t g, uint8_t b) {

    auto colorized_point_cloud = colorizePointCloud(point_cloud, r, g, b);
    auto point_cloud_ros_msg = msg_handler_->convertToROS(colorized_point_cloud);
    publishPointCloud(point_cloud_ros_msg);

}

// Publish Point Cloud
void Visualizer::publishPointCloud(const sensor_msgs::msg::PointCloud2 msg) {

    pointcloud_publisher_->publish(msg);

}

// Colorize Point Cloud
pcl::PointCloud<pcl::PointXYZRGB> Visualizer::colorizePointCloud(const pcl::PointCloud<pcl::PointXYZ> point_cloud, uint8_t r, uint8_t g, uint8_t b) {

    pcl::PointCloud<pcl::PointXYZRGB> colorized_point_cloud;
    colorized_point_cloud.resize(point_cloud.size());
    colorized_point_cloud.header = point_cloud.header;

    for (size_t i = 0; i < point_cloud.size(); ++i) {

        colorized_point_cloud.points[i].x = point_cloud.points[i].x;
        colorized_point_cloud.points[i].y = point_cloud.points[i].y;
        colorized_point_cloud.points[i].z = point_cloud.points[i].z;
            
        colorized_point_cloud.points[i].r = r;
        colorized_point_cloud.points[i].g = g;
        colorized_point_cloud.points[i].b = b;
    }

    return colorized_point_cloud;

}
