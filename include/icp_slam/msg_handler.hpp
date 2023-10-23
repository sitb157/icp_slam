#ifndef MSG_HANDLER_HPP_
#define MSG_HANDLER_HPP_

/**
 * @brief storage for msgs 
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <deque>


class MsgHandler {
    public:

        MsgHandler();

        // Insert data from callback
        void insertPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
        
        // Get xyz Data    
        pcl::PointCloud<pcl::PointXYZ>::Ptr getConvertedPointCloud(); 

        // Convert Point Cloud Msg to XYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Get pointcloud queue size 
        size_t getPointCloudQueueSize();

    private:
        
        // Storage for point cloud msgs
        std::shared_ptr<std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr>> point_cloud_queue_;

};


#endif // MSG_HANDLER_HPP_

