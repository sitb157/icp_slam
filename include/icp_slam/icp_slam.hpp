#ifndef ICP_SLAM_HPP_
#define ICP_SLAM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class ICP_SLAM : public rclcpp::Node {
    public:
        ICP_SLAM(const std::string &node_name);
    private:
        void pointcloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
};

#endif // ICP_SLAM_HPP_
