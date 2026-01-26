#include "follow_controller/follow_node.hpp"

FollowNode::FollowNode() : Node("follow_controller") {
    // Declare Parameters
    this->declare_parameter("target_topic", "/model/leader_sphere/pose");
    this->declare_parameter("cmd_topic", "/cmd_vel");
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("kp_linear", 1.0);
    this->declare_parameter("kd_linear", 0.1);
    this->declare_parameter("kp_angular", 2.0);
    this->declare_parameter("kd_angular", 0.1);

    // Get Parameters
    std::string target_topic = this->get_parameter("target_topic").as_string();
    std::string cmd_topic = this->get_parameter("cmd_topic").as_string();
    control_freq_ = this->get_parameter("control_frequency").as_double();
    kp_linear_ = this->get_parameter("kp_linear").as_double();
    kd_linear_ = this->get_parameter("kd_linear").as_double();
    kp_angular_ = this->get_parameter("kp_angular").as_double();
    kd_angular_ = this->get_parameter("kd_angular").as_double();

    // Pubs/Subs
    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_topic, 10, std::bind(&FollowNode::target_callback, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    // Timer for control loop
    auto period = std::chrono::duration<double>(1.0 / control_freq_);
    timer_ = this->create_wall_timer(period, std::bind(&FollowNode::control_loop, this));
}

void FollowNode::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target_pose_ = msg;
}

void FollowNode::control_loop() {
    if (!target_pose_) return;

    double dx = target_pose_->pose.position.x;
    double dy = target_pose_->pose.position.y;

    // 1. Position Error (Distance to target)
    double dist_error = std::sqrt(dx * dx + dy * dy);
    double d_dist = (dist_error - last_dist_error_) * control_freq_;
    double v = (kp_linear_ * dist_error) + (kd_linear_ * d_dist);

    // 2. Heading Error (Angle to target)
    double target_angle = std::atan2(dy, dx);
    double angle_error = target_angle; // Relative to robot's 0 heading
    double d_angle = (angle_error - last_angle_error_) * control_freq_;
    double w = (kp_angular_ * angle_error) + (kd_angular_ * d_angle);

    // Publish Twist
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::max(0.0, v); // Prevent robot from moving backward if not needed
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);

    // Save states
    last_dist_error_ = dist_error;
    last_angle_error_ = angle_error;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowNode>());
    rclcpp::shutdown();
    return 0;
}