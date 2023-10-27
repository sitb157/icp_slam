#include "icp_slam/visualizer.hpp"

// Initialize
Visualizer::Visualizer(const std::string &node_name) : Node(node_name) {

    // Publisher
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("result_point_cloud", 10);
    
    // Set parameter for visualizer

    // Massages handler
    msg_handler_ = std::make_shared<MsgHandler>();

}

// Publish Point Cloud
void Visualizer::publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    auto point_cloud_ros_msg = msg_handler_->convertToROS(point_cloud);
    pointcloud_publisher_->publish(point_cloud_ros_msg);
    RCLCPP_INFO(this->get_logger(), "publish pointcloud");
}
