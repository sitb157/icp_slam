#include "icp_slam/msg_handler.hpp"

MsgHandler::MsgHandler() {
    //Initialize
    pointcloudQueue_ = std::make_shared<std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>>();
}

void MsgHandler::insertPointCloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pointcloudQueue_->push_back(msg);
}

std::size_t MsgHandler::getPointCloudQueueSize() {
    return pointcloudQueue_->size();
}
