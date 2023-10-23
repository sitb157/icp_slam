#ifndef MSG_HANDLER_HPP_
#define MSG_HANDLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
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
        size_t getPointCloudQueueSize();

    private:

        std::shared_ptr<std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>> pointcloudQueue_;
        std::shared_ptr<std::vector<std::vector<int>>> xyzVectors_;

        // Convert Point Cloud Msg to XYZ
        std::vector<std::vector<float>> convertToXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};


#endif // MSG_HANDLER_HPP_

