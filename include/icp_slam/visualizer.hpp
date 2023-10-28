#ifndef VISUALIZER_HPP_
#define VISUALIZER_HPP_

/**
 * @brief visualize cloud map and trajectory
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "icp_slam/msg_handler.hpp"

class Visualizer : public rclcpp::Node{

    public:

        // Initialize
        Visualizer(const std::string &node_name);

        // Visualize Point Cloud
        void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ> point_cloud, uint8_t r, uint8_t g, uint8_t b);

    private: 

        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

        // Massages handler
        std::shared_ptr<MsgHandler> msg_handler_;

        // ROS Point Cloud Msg
        sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_ros_msg_;

        // Publish Point Cloud
        void publishPointCloud(sensor_msgs::msg::PointCloud2 colorized_point_cloud);

        // Colorize Point Cloud
        pcl::PointCloud<pcl::PointXYZRGB> colorizePointCloud(const pcl::PointCloud<pcl::PointXYZ> point_cloud, uint8_t r, uint8_t g, uint8_t b);
};

#endif // VISUALIZER_HPP_
