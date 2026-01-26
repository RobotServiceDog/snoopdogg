#include "follow_controller/follow_node.hpp"
#include <cmath>

FollowNode::FollowNode() : Node("follow_controller") {
    // Declare Parameters
    this->declare_parameter("target_topic", "/model/leader_sphere/pose");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("cmd_topic", "/cmd_vel");
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("kp_linear", 1.0);
    this->declare_parameter("kd_linear", 0.2);
    this->declare_parameter("kp_angular", 2.5);
    this->declare_parameter("kd_angular", 0.3);
    this->declare_parameter("goal_tolerance", 0.15);
    this->declare_parameter("max_linear_vel", 1.0);
    this->declare_parameter("max_angular_vel", 2.0);

    control_freq_ = this->get_parameter("control_frequency").as_double();
    kp_linear_ = this->get_parameter("kp_linear").as_double();
    kd_linear_ = this->get_parameter("kd_linear").as_double();
    kp_angular_ = this->get_parameter("kp_angular").as_double();
    kd_angular_ = this->get_parameter("kd_angular").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();

    // Subscribers
    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("target_topic").as_string(), 10, 
        std::bind(&FollowNode::target_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odom_topic").as_string(), 10,
        std::bind(&FollowNode::odom_callback, this, std::placeholders::_1));

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        this->get_parameter("cmd_topic").as_string(), 10);

    // Timer for control loop
    auto period = std::chrono::duration<double>(1.0 / control_freq_);
    timer_ = this->create_wall_timer(period, std::bind(&FollowNode::control_loop, this));
    
    RCLCPP_INFO(this->get_logger(), "Follow controller initialized");
}

void FollowNode::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_ = msg;
}

void FollowNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
}

double FollowNode::normalize_angle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

double FollowNode::quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void FollowNode::control_loop() {
    if (!target_pose_ || !current_odom_) {
        return;
    }

    const double dt = 1.0 / control_freq_;
    
    // ==================== Position Error ====================
    double dx = target_pose_->pose.position.x - current_odom_->pose.pose.position.x;
    double dy = target_pose_->pose.position.y - current_odom_->pose.pose.position.y;
    double distance_error = std::sqrt(dx * dx + dy * dy);
    
    // Distance error derivative using finite difference
    double distance_error_dot = (distance_error - last_dist_error_) / dt;
    
    // ==================== Goal Reached Check ====================
    if (distance_error < goal_tolerance_) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        
        last_dist_error_ = 0.0;
        last_angle_error_ = 0.0;
        return;
    }
    
    // ==================== Angular Error ====================
    double robot_yaw = quaternion_to_yaw(current_odom_->pose.pose.orientation);
    double desired_yaw = std::atan2(dy, dx);
    double angle_error = normalize_angle(desired_yaw - robot_yaw);
    
    // Angular error derivative using finite difference
    double angle_error_dot = normalize_angle(angle_error - last_angle_error_) / dt;
    
    // ==================== PD Control ====================
    // Linear velocity: PD controller on distance error
    double alignment = std::cos(angle_error);
    double v_pd = kp_linear_ * distance_error + kd_linear_ * distance_error_dot;
    double v = v_pd * std::max(0.0, alignment);
    
    // Angular velocity: PD controller on angle error
    double w = kp_angular_ * angle_error + kd_angular_ * angle_error_dot;
    
    // ==================== Velocity Limits ====================
    v = std::clamp(v, 0.0, max_linear_vel_);
    w = std::clamp(w, -max_angular_vel_, max_angular_vel_);
    
    // ==================== Publish Command ====================
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
    
    // // ==================== Debug Output ====================
    // static int counter = 0;
    // if (++counter % 50 == 0) {
    //     RCLCPP_INFO(this->get_logger(), 
    //                 "dist: %.3fm (%.3fm/s), angle: %.1f° (%.1f°/s), v: %.2f, w: %.2f", 
    //                 distance_error, distance_error_dot,
    //                 angle_error * 180.0 / M_PI, angle_error_dot * 180.0 / M_PI,
    //                 v, w);
    // }
    
    // ==================== Update Previous States ====================
    last_dist_error_ = distance_error;
    last_angle_error_ = angle_error;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowNode>());
    rclcpp::shutdown();
    return 0;
}