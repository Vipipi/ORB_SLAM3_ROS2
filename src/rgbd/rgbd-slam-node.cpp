#include "rgbd-slam-node.hpp"
#include <opencv2/core/core.hpp>

RgbdSlamNode::RgbdSlamNode(const std::string& voc_path, const std::string& settings_path)
    :   Node("ORB_SLAM3_ROS2"),
        m_SLAM(new ORB_SLAM3::System(voc_path, settings_path, ORB_SLAM3::System::RGBD, true))
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/rgb", qos_profile.get_rmw_qos_profile());
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/depth", qos_profile.get_rmw_qos_profile());

    // Create the synchronizer
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    
    // Set the slop by creating an explicit rclcpp::Duration object
    syncApproximate->setSlop(rclcpp::Duration::from_seconds(0.1));

    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);
}

RgbdSlamNode::~RgbdSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
}
