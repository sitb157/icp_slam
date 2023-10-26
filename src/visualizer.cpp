#include "icp_slam/visualizer.hpp"

// Initialize
Visualizer::Visualizer(const std::string &node_name) : Node(node_name) {

    // Publisher
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("result_point_cloud", 10);
    
    // Set parameter for visualizer

    // Massages handler
    msg_handler_ = std::make_shared<MsgHandler>();

}

void Visualizer::publishPointCloud() {
    RCLCPP_INFO(this->get_logger(), "publish pointcloud");
}
