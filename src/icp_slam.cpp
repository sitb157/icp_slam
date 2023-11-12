#include "icp_slam/icp_slam.hpp"

// Initialize
ICP_SLAM::ICP_SLAM(const std::string &node_name) : Node(node_name) {

    // Subscriber 
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("diff_drive/pointcloud", 10, std::bind(&ICP_SLAM::pointcloudCallBack, this, std::placeholders::_1));
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("diff_drive/imu", 10, std::bind(&ICP_SLAM::imuCallBack, this, std::placeholders::_1));

    // Current and Previous Point Cloud
    curr_point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    prev_point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // Vertex id
    id_ = -1;

    // point cloud Visualizer
    visualizer_ = std::make_shared<Visualizer>(node_name + "_visualizer");

    // point cloud Visualizer for debugging
    result_visualizer_ = std::make_shared<Visualizer>(node_name + "_result_visualizer");
 
    // Msg Handler
    msg_handler_ = std::make_shared<MsgHandler>();

    // Graph Optimizer
    graph_optimizer_ = std::make_shared<GraphOptimizer>();


    // ICP
    icp_ = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();

    // Set parameter for icp
    icp_->setMaxCorrespondenceDistance(1.0);
    icp_->setTransformationEpsilon(0.001);
    icp_->setMaximumIterations(1000);
    icp_->setEuclideanFitnessEpsilon(1);



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

    auto curr_downsampled_point_cloud = msg_handler_->downSamplingPointCloud(curr_point_cloud_, 0.1f);
    auto prev_downsampled_point_cloud = msg_handler_->downSamplingPointCloud(prev_point_cloud_, 0.1f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> curr_indices;
    std::vector<int> prev_indices;
    pcl::removeNaNFromPointCloud(*curr_downsampled_point_cloud, *curr_downsampled_point_cloud, curr_indices);
    pcl::removeNaNFromPointCloud(*prev_downsampled_point_cloud, *prev_downsampled_point_cloud, prev_indices);
    icp_->setInputSource(curr_downsampled_point_cloud);
    icp_->setInputTarget(prev_downsampled_point_cloud);
    icp_->align(*align);

    if (icp_.hasConverged()) {
        // If ICP converged, the transformation matrix can be retrieved
        Eigen::Matrix4f icp_transformation = icp_.getFinalTransformation();
        // Now you can use icp_transformation for further processing
        //
        // Convert Eigen::Matrix4f (ICP output) to Eigen::Matrix4d
        Eigen::Matrix4d icp_transformation_d = icp_transformation.cast<double>();

        // Extract rotation (as a 3x3 matrix) and translation (as a 3x1 vector)
        Eigen::Matrix3d rotation = icp_transformation_d.block<3,3>(0,0);
        Eigen::Vector3d translation = icp_transformation_d.block<3,1>(0,3);
        
        // Create a g2o::SE3Quat from the rotation and translation
        g2o::SE3Quat pose_estimate(rotation, translation);
        graph_optimizer_->addVertex(pose_estimate, id_);
    } else {
        std::cerr << "ICP did not converge." << std::endl;
    }
    visualizer_->visualizePointCloud(*curr_point_cloud_, 255, 0, 255);
    result_visualizer_->visualizePointCloud(*align, 0, 0, 255);

}

void ICP_SLAM::backEnd() {
    while (rclcpp::ok()) {
        //RCLCPP_INFO(this->get_logger(), "Run Thread");
    }
}
