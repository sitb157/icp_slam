#ifndef ICP_SLAM_HPP_
#define ICP_SLAM_HPP_

/**
 * @brief icp slam
 */

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "icp_slam/msg_handler.hpp"
#include "icp_slam/icp.hpp"

class ICP_SLAM : public rclcpp::Node {
    public:
        ICP_SLAM(const std::string &node_name);
        ~ICP_SLAM();

        void startThread();
        void joinThread();

    private:

        void pointcloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg);


        void frontEnd();

        // Thread 
        void backEnd();

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_point_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_point_cloud_;

        std::shared_ptr<MsgHandler> msg_handler_;
        std::shared_ptr<ICP> icp_;
        
        std::thread thread_;

};

#endif // ICP_SLAM_HPP_
