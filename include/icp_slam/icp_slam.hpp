#ifndef ICP_SLAM_HPP_
#define ICP_SLAM_HPP_

/**
 * @brief icp slam
 */

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "icp_slam/visualizer.hpp"
#include "icp_slam/msg_handler.hpp"
#include "icp_slam/graph_optimizer.hpp"
#include <pcl/registration/icp.h>
//#include "icp_slam/icp.hpp"

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
        size_t id_;

        // point cloud Visualizer
        std::shared_ptr<Visualizer> visualizer_;

        // point cloud Visualizer for debugging
        std::shared_ptr<Visualizer> result_visualizer_;

        // Data handler
        std::shared_ptr<MsgHandler> msg_handler_;

        // Graph Optimizer in backend
        std::shared_ptr<GraphOptimizer> graph_optimizer_;
        
        // Backend Thread
        std::thread back_end_thread_;

        // ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp_;
};

#endif // ICP_SLAM_HPP_
