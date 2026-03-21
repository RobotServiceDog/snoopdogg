#ifndef FOLLOW_CONTROLLER__FOLLOW_NODE_HPP_
#define FOLLOW_CONTROLLER__FOLLOW_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"  // Added for error publishing
#include "nav_msgs/msg/odometry.hpp"

class FollowNode : public rclcpp::Node {
public:
    FollowNode();

private:
    // Callbacks
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_loop();

    // Helper functions
    double normalize_angle(double angle);
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q);

    // Subscribers & Publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr error_pub_; // Added

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    geometry_msgs::msg::PoseStamped::SharedPtr target_pose_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    double last_dist_error_ = 0.0;
    double last_angle_error_ = 0.0;

    // Parameters
    double control_freq_;
    double kp_linear_, kd_linear_;
    double kp_angular_, kd_angular_;
    double goal_tolerance_, goal_angle_tolerance_;
    double max_linear_vel_, max_angular_vel_;
};

#endif  // FOLLOW_CONTROLLER__FOLLOW_NODE_HPP_