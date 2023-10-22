#include "icp_slam/icp_slam.hpp"

ICP_SLAM::ICP_SLAM(const std::string &node_name) : Node(node_name) {
    // Initialize
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("diff_drive/pointcloud", 10, std::bind(&ICP_SLAM::pointcloudCallBack, this, std::placeholders::_1));
}

void ICP_SLAM::pointcloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received message");
}
