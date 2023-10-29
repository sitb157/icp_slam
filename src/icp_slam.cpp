#include "icp_slam/icp_slam.hpp"


// Initialize
ICP_SLAM::ICP_SLAM(const std::string &node_name) : Node(node_name) {

    // Subscriber 
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("diff_drive/pointcloud", 10, std::bind(&ICP_SLAM::pointcloudCallBack, this, std::placeholders::_1));
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("diff_drive/imu", 10, std::bind(&ICP_SLAM::imuCallBack, this, std::placeholders::_1));

    // Current and Previous Point Cloud
    curr_point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    prev_point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Visualizer
    visualizer_ = std::make_shared<Visualizer>(node_name + "_visualizer");
    // Msg Handler
    msg_handler_ = std::make_shared<MsgHandler>();
    // ICP 
    icp_ = std::make_shared<ICP>(); 

    // BackEnd Thread
    back_end_thread_ = std::thread(&ICP_SLAM::backEnd, this);
}

ICP_SLAM::~ICP_SLAM() {

    if (back_end_thread_.joinable()) {
        back_end_thread_.join();
    }

}



// Subscribe point cloud msgs
// Get Transformation between current point cloud and previous point cloud
void ICP_SLAM::pointcloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    //RCLCPP_INFO(this->get_logger(), "Received pointcloud message");

    curr_point_cloud_ = msg_handler_->convertToPCL(msg);
    
    RCLCPP_INFO(this->get_logger(), "point cloud size is %ld", msg_handler_->getPointCloudQueueSize());

    msg_handler_->insertPointCloud(curr_point_cloud_);
    if (prev_point_cloud_->empty()) { 

    } else {
        frontEnd();
    }
    pcl::copyPointCloud(*curr_point_cloud_, *prev_point_cloud_); 
    //auto point_cloud = msg_handler_->getConvertedPointCloud();
}

void ICP_SLAM::imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received imu message");
}

// Get transformation between current point cloud and previous point cloud and Add factor into pose graph manager
void ICP_SLAM::frontEnd() {

    auto cloud_downsampled = msg_handler_->downSamplingPointCloud(curr_point_cloud_, 0.1f);
    visualizer_->visualizePointCloud(*cloud_downsampled, 255, 0, 255);
    //auto transformation = icp_->getTransformation(curr_point_cloud_, prev_point_cloud_);
}

void ICP_SLAM::backEnd() {
    while (rclcpp::ok()) {
        //RCLCPP_INFO(this->get_logger(), "Run Thread");
    }
}
