/*
* RGBD mode example for ORB-SLAM3 with RealSense D405
* Author: Based on mono_example.cpp by Azmyin Md. Kamal
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
    auto node = std::make_shared<RGBDMode>();

    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------
