#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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
    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    std::unique_ptr<ORB_SLAM3::System> m_SLAM;

    // --- CHANGE IS HERE ---
    // Changed CvImagePtr to CvImageConstPtr
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;
    // --- END OF CHANGE ---

    using ImageMsg = sensor_msgs::msg::Image;
    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

    std::shared_ptr<message_filters::Subscriber<ImageMsg>> rgb_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> depth_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;
};

#endif
