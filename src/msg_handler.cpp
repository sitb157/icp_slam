#include "icp_slam/msg_handler.hpp"

MsgHandler::MsgHandler() {
    //Initialize
    pointcloudQueue_ = std::make_shared<std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>>();
}

void MsgHandler::insertPointCloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pointcloudQueue_->push_back(msg);
}

size_t MsgHandler::getPointCloudQueueSize() {
    return pointcloudQueue_->size();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MsgHandler::convertToXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    const size_t num_points = msg->height * msg->width;
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    // Insert xyz data
    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        point_cloud->push_back(pcl::PointXYZ(*iter_x, *iter_y, *iter_z));
    } 

    return point_cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr MsgHandler::getXYZ() {

    // Get data and pop
    const sensor_msgs::msg::PointCloud2::SharedPtr output = pointcloudQueue_->front();
    pointcloudQueue_->pop_front(); 

    return convertToXYZ(output);
}
