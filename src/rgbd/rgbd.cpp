#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    
    // Create the node, passing file paths to the constructor
    auto node = std::make_shared<RgbdSlamNode>(argv[1], argv[2]);

    // Spin the node to process callbacks
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
