#include "icp_slam/msg_handler.hpp"

// Initialize
MsgHandler::MsgHandler() {
    point_cloud_queue_ = std::make_shared<std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr>>();
}

void MsgHandler::insertPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    point_cloud_queue_->push_back(point_cloud);
}

size_t MsgHandler::getPointCloudQueueSize() {
    return point_cloud_queue_->size();
}


// Convert ros msgs to point cloud of pcl 
pcl::PointCloud<pcl::PointXYZ>::Ptr MsgHandler::convertToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr pt_msg) {

    pcl::PointCloud<pcl::PointXYZ> pt_dst;
    pcl::fromROSMsg(*pt_msg, pt_dst);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr pt_dst(new pcl::PointCloud<pcl::PointXYZ>);
    //const size_t num_points = msg->height * msg->width;
    //sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    //sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    //sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    //// Insert xyz data
    //for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    //    pt_dst->push_back(pcl::PointXYZ(*iter_x, *iter_y, *iter_z));
    //} 

    return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pt_dst);
}

// Convert point cloud to ros msgs
sensor_msgs::msg::PointCloud2 MsgHandler::convertToROS(const pcl::PointCloud<pcl::PointXYZ> src) {

    sensor_msgs::msg::PointCloud2 dst;
    pcl::toROSMsg(src, dst);

    return dst;
}

// Convert point cloud to ros msgs
sensor_msgs::msg::PointCloud2 MsgHandler::convertToROS(const pcl::PointCloud<pcl::PointXYZRGB> src) {

    sensor_msgs::msg::PointCloud2 dst;
    pcl::toROSMsg(src, dst);

    return dst;
}

// Point Cloud Down sampling with voxel filter 
pcl::PointCloud<pcl::PointXYZ>::Ptr MsgHandler::downSamplingPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr src, float voxel_size = 0.05f) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr dst(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(src);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*dst);

    return dst;
}



// Get pcl Data in queue 
pcl::PointCloud<pcl::PointXYZ>::Ptr MsgHandler::getConvertedPointCloud() {

    // Get data and pop
    auto  output = point_cloud_queue_->front();
    point_cloud_queue_->pop_front(); 

    return output;
}
