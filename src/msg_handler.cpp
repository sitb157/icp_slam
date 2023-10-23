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

std::vector<std::vector<float>> MsgHandler::convertToXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    const size_t num_points = msg->height * msg->width;
    std::vector<std::vector<float>> xyz_output;
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        xyz_output.push_back(std::vector<float>{*iter_x, *iter_y, *iter_z});
        
    } 
    return xyz_output;
}
