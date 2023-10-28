#include "icp_slam/icp_slam.hpp"


// Initialize
ICP_SLAM::ICP_SLAM(const std::string &node_name) : Node(node_name) {

    // Subscriber 
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("diff_drive/pointcloud", 10, std::bind(&ICP_SLAM::pointcloudCallBack, this, std::placeholders::_1));
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("diff_drive/imu", 10, std::bind(&ICP_SLAM::imuCallBack, this, std::placeholders::_1));

    // Previous Point Cloud
    prev_point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

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

    auto curr_point_cloud_ = msg_handler_->convertToPCL(msg);
    
    RCLCPP_INFO(this->get_logger(), "point cloud size is %ld", msg_handler_->getPointCloudQueueSize());

    msg_handler_->insertPointCloud(curr_point_cloud_);
    if (prev_point_cloud_->empty()) { 

    } else {
      //frontEnd();
    }
    pcl::copyPointCloud(curr_point_cloud_, *prev_point_cloud_); 
    //auto point_cloud = msg_handler_->getConvertedPointCloud();
    visualizer_->visualizePointCloud(curr_point_cloud_, 255, 255, 0);
}

void ICP_SLAM::imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received imu message");
}

// Get transformation between current point cloud and previous point cloud and Add factor into pose graph manager
void ICP_SLAM::frontEnd() {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    //sor.setInputCloud(curr_point_cloud_);
    //sor.setLeafSize(0.01f, 0.01f, 0.01f);
    ////pcl::PointCloud
    //auto transformation = icp_->getTransformation(curr_point_cloud_, prev_point_cloud_);
}
