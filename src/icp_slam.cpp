#include "icp_slam/icp_slam.hpp"


// Initialize
ICP_SLAM::ICP_SLAM(const std::string &node_name) : Node(node_name) {

    // Subscriber 
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("diff_drive/pointcloud", 10, std::bind(&ICP_SLAM::pointcloudCallBack, this, std::placeholders::_1));
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("diff_drive/imu", 10, std::bind(&ICP_SLAM::imuCallBack, this, std::placeholders::_1));

    // Current and Previous Point Cloud
    prev_point_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    curr_point_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // Visualizer
    visualizer_ = std::make_shared<Visualizer>(node_name + "_visualizer");
    // Msg Handler
    msg_handler_ = std::make_shared<MsgHandler>();
    // ICP 
    icp_ = std::make_shared<ICP>(); 
}



// Subscribe point cloud msgs
// Get Transformation between current point cloud and previous point cloud
void ICP_SLAM::pointcloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "Received pointcloud message");
    auto curr_point_cloud_ = msg_handler_->convertToXYZ(msg);
    RCLCPP_INFO(this->get_logger(), "point cloud size is %ld", msg_handler_->getPointCloudQueueSize());

    msg_handler_->insertPointCloud(curr_point_cloud_);
    if (prev_point_cloud_->empty()) { 

    } else {
        frontEnd();
        
        
    }
    pcl::copyPointCloud(*curr_point_cloud_, *prev_point_cloud_); 
    auto point_cloud = msg_handler_->getConvertedPointCloud();
    //for (pcl::PointCloud<pcl::PointXYZ>::iterator it = point_cloud->begin(); it != point_cloud->end(); ++ it) {
    //    RCLCPP_INFO(get_logger(), "point cloud x = %f, y = %f, z = %f", it->x, it->y, it->z);
    //}
}

void ICP_SLAM::imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received imu message");
}

// Get transformation between current point cloud and previous point cloud and Add factor into pose graph manager
void ICP_SLAM::frontEnd() {
    auto transformation = icp_->getTransformation(curr_point_cloud_, prev_point_cloud_);
}
