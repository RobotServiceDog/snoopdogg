#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ApriltagDetectionLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit ApriltagDetectionLifecycleNode(const rclcpp::NodeOptions &options);

    protected:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:

        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

        // ROS elements
        geometry_msgs::msg::Pose2D apriltag_pose_msg_;

        std::string image_topic_;
        std::string camera_info_topic_;
        double tag_size_m_;
        int target_tag_id_;
        bool use_camera_info_topic_;
        std::vector<double> camera_matrix_param_;
        std::vector<double> distortion_coefficients_param_;
        int calibration_width_;
        int calibration_height_;

        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        bool has_camera_info_;
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;

        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose2D>::SharedPtr apriltag_pose_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
};
