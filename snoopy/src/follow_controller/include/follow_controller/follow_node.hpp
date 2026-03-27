#ifndef FOLLOW_NODE_HPP_
#define FOLLOW_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp" // Added for ground truth

class FollowNode : public rclcpp::Node {
public:
    FollowNode();

private:
    void target_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_loop();

    double normalize_angle(double angle);
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q);
    
    // Subscribers and Publishers
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data storage
    geometry_msgs::msg::Pose2D::SharedPtr target_pose_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    
    // Controller variables
    double kp_linear_, kd_linear_;
    double kp_angular_, kd_angular_;
    double last_dist_error_ = 0.0;
    double last_angle_error_ = 0.0;
    double control_freq_;
    double goal_tolerance_;
    double goal_angle_tolerance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double deadzone_theta_;
    double angular_offset_;
};

#endif