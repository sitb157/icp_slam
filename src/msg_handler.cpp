#include "icp_slam/msg_handler.hpp"

// Initialize
MsgHandler::MsgHandler() {
    point_cloud_queue_ = std::make_shared<std::deque<pcl::PointCloud<pcl::PointXYZ>>>();
}

void MsgHandler::insertPointCloud(const pcl::PointCloud<pcl::PointXYZ> point_cloud) {
    point_cloud_queue_->push_back(point_cloud);
}

size_t MsgHandler::getPointCloudQueueSize() {
    return point_cloud_queue_->size();
}


// Convert ros msgs to point cloud of pcl 
pcl::PointCloud<pcl::PointXYZ> MsgHandler::convertToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr pt_msg) {

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

    return pt_dst;
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

// Get pcl Data in queue 
pcl::PointCloud<pcl::PointXYZ> MsgHandler::getConvertedPointCloud() {

    // Get data and pop
    auto  output = point_cloud_queue_->front();
    point_cloud_queue_->pop_front(); 

    return output;
}
