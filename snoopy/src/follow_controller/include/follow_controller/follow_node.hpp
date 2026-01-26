#ifndef FOLLOW_NODE_HPP_
#define FOLLOW_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

class FollowNode : public rclcpp::Node {
public:
    FollowNode();

private:
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void control_loop();

    // ROS 2 Interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    geometry_msgs::msg::PoseStamped::SharedPtr target_pose_;
    
    // PD Gains and Parameters
    double kp_linear_, kd_linear_;
    double kp_angular_, kd_angular_;
    double last_dist_error_ = 0.0;
    double last_angle_error_ = 0.0;
    double control_freq_;
};

#endif