#ifndef ICP_SLAM_HPP_
#define ICP_SLAM_HPP_

/**
 * @brief icp slam
 */

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "icp_slam/visualizer.hpp"
#include "icp_slam/msg_handler.hpp"
#include "icp_slam/icp.hpp"

class ICP_SLAM : public rclcpp::Node {

    public:

        ICP_SLAM(const std::string &node_name);
        ~ICP_SLAM();

    private:

        void pointcloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg);


        void frontEnd();

        // Backend Function 
        void backEnd();

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_point_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_point_cloud_;

        std::shared_ptr<Visualizer> visualizer_;
        std::shared_ptr<MsgHandler> msg_handler_;
        std::shared_ptr<ICP> icp_;
        
        // Backend Thread
        std::thread back_end_thread_;
        
};

#endif // ICP_SLAM_HPP_
