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

        Visualizer(const std::string &node_name);

        void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    private: 

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

        std::shared_ptr<MsgHandler>  msg_handler_;

};

#endif // VISUALIZER_HPP_
