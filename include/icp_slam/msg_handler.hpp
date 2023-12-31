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
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <deque>


class MsgHandler {
    public:

        // Initialize
        MsgHandler();

        // Insert data from callback
        void insertPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

        // Get xyz Data    
        pcl::PointCloud<pcl::PointXYZ>::Ptr getConvertedPointCloud(); 
        // Point Cloud Down sampling with voxel filter 
        pcl::PointCloud<pcl::PointXYZ>::Ptr downSamplingPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr src, float voxel_size);

        // Convert ros point cloud msgs to pcl point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr pt_msg);

        // Convert pcl point cloud to ros point cloud msgs 
        sensor_msgs::msg::PointCloud2 convertToROS(const pcl::PointCloud<pcl::PointXYZ> src);

        // Convert pcl point cloud to ros point cloud msgs 
        sensor_msgs::msg::PointCloud2 convertToROS(const pcl::PointCloud<pcl::PointXYZRGB> src);

        // Get pointcloud queue size 
        size_t getPointCloudQueueSize();

        // Storage for point cloud msgs
        std::shared_ptr<std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr>> point_cloud_queue_;

    private:
        
};


#endif // MSG_HANDLER_HPP_

