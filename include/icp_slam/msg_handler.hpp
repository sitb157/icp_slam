#ifndef MSG_HANDLER_HPP_
#define MSG_HANDLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <memory>
#include <deque>

/**
 * @brief storage for msgs 
 */


class MsgHandler {
    public:

        MsgHandler();

        // Insert data from callback
        void insertPointCloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Get pointcloud queue size 
        std::size_t getPointCloudQueueSize();

    private:

        std::shared_ptr<std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>> pointcloudQueue_;
};


#endif // MSG_HANDLER_HPP_

