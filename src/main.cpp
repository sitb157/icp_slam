#include "icp_slam/icp_slam.hpp"

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto icp_slam_node = std::make_shared<ICP_SLAM>("icp_slam_node");

    rclcpp::spin(icp_slam_node);
    rclcpp::shutdown();

    return 0;
}

