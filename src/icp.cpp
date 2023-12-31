#include "icp_slam/icp.hpp"

// Initialize
ICP::ICP() {

    // Set parameter for icp
    icp_lib_.setMaxCorrespondenceDistance(1.0);
    icp_lib_.setTransformationEpsilon(0.001);
    icp_lib_.setMaximumIterations(1000);
    icp_lib_.setEuclideanFitnessEpsilon(1);

}

// Get transformation between src and dst
//Eigen::Matrix4f ICP::getTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr dst) {
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr align;
//    icp_lib_.setInputSource(src);
//    icp_lib_.setInputTarget(dst);
//    icp_lib_.align(*align);
//
//    return icp_lib_.getFinalTransformation();
//}

// Get transformation between src and dst
void ICP::getTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr dst) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> src_indices;
    std::vector<int> dst_indices;
    pcl::removeNaNFromPointCloud(*src, *src, src_indices);
    pcl::removeNaNFromPointCloud(*dst, *dst, dst_indices);
    icp_lib_.setInputSource(src);
    icp_lib_.setInputTarget(dst);
    icp_lib_.align(*align);

}

