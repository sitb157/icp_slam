#ifndef ICP_HPP_
#define ICP_HPP_

/**
 * @brief find transformation with icp
 */

#include <pcl/point_types.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

class ICP {
    public:
        ICP();
        
        Eigen::Matrix4f getTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr dst);

    private:

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_lib_;
};

#endif // ICP_HPP_
