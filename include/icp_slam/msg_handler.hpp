#ifndef MSG_HANDLER_HPP_
#define MSG_HANDLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
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
        
        // Get xyz Data    
        pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZ(); 


        // Get pointcloud queue size 
        size_t getPointCloudQueueSize();

    private:
        
        // Storage for point cloud msgs
        std::shared_ptr<std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>> pointcloudQueue_;

        // Convert Point Cloud Msg to XYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};


#endif // MSG_HANDLER_HPP_

