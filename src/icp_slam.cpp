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
    RCLCPP_INFO(get_logger(), "point cloud size is %ld", msg_handler_->getPointCloudQueueSize());
    auto point_cloud = msg_handler_->getXYZ();
    //for (pcl::PointCloud<pcl::PointXYZ>::iterator it = point_cloud->begin(); it != point_cloud->end(); ++ it) {
    //    RCLCPP_INFO(get_logger(), "point cloud x = %f, y = %f, z = %f", it->x, it->y, it->z);
    //}
}

void ICP_SLAM::imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg) {
    //RCLCPP_INFO(get_logger(), "Received imu message");
}
