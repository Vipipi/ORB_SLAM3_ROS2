#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp" // Add CameraInfo message
#include "cv_bridge/cv_bridge.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <memory>

#include "System.h"
#include "utility.hpp"

class RgbdSlamNode : public rclcpp::Node
{
public:
    RgbdSlamNode(const std::string &voc_path, const std::string &settings_path);
    ~RgbdSlamNode();

private:
    // Change callback to accept three messages
    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, 
                  const sensor_msgs::msg::Image::SharedPtr msgD,
                  const sensor_msgs::msg::CameraInfo::SharedPtr msg_info);

    std::unique_ptr<ORB_SLAM3::System> m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    using ImageMsg = sensor_msgs::msg::Image;
    using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
    // Change sync policy to handle three topics
    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, CameraInfoMsg>;

    std::shared_ptr<message_filters::Subscriber<ImageMsg>> rgb_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> depth_sub;
    std::shared_ptr<message_filters::Subscriber<CameraInfoMsg>> info_sub; // Add subscriber for info

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;
};

#endif
