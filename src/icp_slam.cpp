#include "icp_slam/icp_slam.hpp"

ICP_SLAM::ICP_SLAM(const std::string &node_name) : Node(node_name) {

    // Initialize
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("diff_drive/pointcloud", 10, std::bind(&ICP_SLAM::pointcloudCallBack, this, std::placeholders::_1));
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("diff_drive/imu", 10, std::bind(&ICP_SLAM::imuCallBack, this, std::placeholders::_1));

    // Msg Handler
    msg_handler_ = std::make_shared<MsgHandler>();
}

void ICP_SLAM::pointcloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received pointcloud message");
    msg_handler_->insertPointCloudMsg(msg);
    RCLCPP_INFO(get_logger(), "pooint cloud size is %ld", msg_handler_->getPointCloudQueueSize());
    
}

void ICP_SLAM::imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg) {
    //RCLCPP_INFO(get_logger(), "Received imu message");
}
