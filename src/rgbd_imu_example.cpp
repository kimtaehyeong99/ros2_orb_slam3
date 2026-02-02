/*
* RGBD + IMU mode example for ORB-SLAM3 with RealSense D405 and external IMU
* Author: Based on rgbd_example.cpp
* Date: 2026
* Compatible for ROS2 Jazzy
*/

//* Import all necessary modules
#include "ros2_orb_slam3/common.hpp"

//* main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Always the first line, initialize this node

    //* Declare a node object
    auto node = std::make_shared<IMU_RGBDMode>();

    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------
