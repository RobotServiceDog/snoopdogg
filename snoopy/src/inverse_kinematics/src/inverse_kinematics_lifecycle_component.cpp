
#include "inverse_kinematics/inverse_kinematics_lifecycle_component.hpp"

void InverseKinematicsLifecycleNode::leg_inverse_kinematics_(int leg_side_constant, double x_pos, double y_pos, double z_pos)
{
    // Write xyz wrt virtual hip joint
    x_pos = x_pos - ik_config_.hip_offset_x;
    z_pos = z_pos - ik_config_.hip_offset_z;
    std::cout << "Target position wrt hip: (" << x_pos << ", " << y_pos << ", " << z_pos << ")" << std::endl;
    
    // Find theta1
    double dyz = sqrt(pow(y_pos, 2) + pow(z_pos, 2));
    double lyz = sqrt(pow(dyz, 2) - pow(ik_config_.h, 2));
    double gamma_yz = -atan(y_pos/z_pos);
    double gamma_h = -atan(ik_config_.h/lyz);
    // leg_side_constant is +1 for left leg, -1 for right leg
    double theta1 = gamma_yz + leg_side_constant * gamma_h;
    std::cout << "theta1: " << theta1 << std::endl;
    joint_angles_.push_back(theta1);

    // Find theta3
    double lxz = sqrt(pow(x_pos, 2) + pow(lyz, 2));
    double n = (pow(lxz, 2) - pow(ik_config_.hu, 2) - pow(ik_config_.hl, 2)) / (2 * ik_config_.hu);
    double arg_theta3 = std::clamp(n / ik_config_.hl, -1.0 + 1e-5, 1.0 - 1e-5);
    double theta3 = -acos(arg_theta3) - ik_config_.servo_angle_offsets[{environment_, "theta_3"}];
    std::cout << "offset: " << ik_config_.servo_angle_offsets[{environment_, "theta_3"}] << std::endl;
    std::cout << "theta3 (raw): " << -acos(arg_theta3) << std::endl;
    std::cout << "theta3: " << theta3 << std::endl;

    // Find theta2
    double alpha_xz = -atan(x_pos/lyz);
    double arg_alpha_off = std::clamp((ik_config_.hu + n) / lxz, -1.0 + 1e-5, 1.0 - 1e-5);
    double alpha_off = acos(arg_alpha_off);
    double theta2 = alpha_xz + alpha_off - ik_config_.servo_angle_offsets[{environment_, "theta_2"}];
    std::cout << "offset: " << ik_config_.servo_angle_offsets[{environment_, "theta_2"}] << std::endl;
    std::cout << "theta2 (raw): " << alpha_xz + alpha_off << std::endl;
    std::cout << "theta2: " << theta2 << std::endl;
    joint_angles_.push_back(theta2);
    joint_angles_.push_back(theta3);

    std::cout << std::endl;
}

void InverseKinematicsLifecycleNode::leg_position_callback_(const comm_utils::msg::LegPosition::SharedPtr msg)
{
    // Clear previous data
    joint_angles_.clear();

    // Process incoming message
    environment_ = msg->env;
    foot_positions_ = msg->foot_position;

    leg_inverse_kinematics_(msg->LEFT_LEG_CONSTANT, foot_positions_[0], foot_positions_[1], foot_positions_[2]);
    leg_inverse_kinematics_(msg->RIGHT_LEG_CONSTANT, foot_positions_[3], foot_positions_[4], foot_positions_[5]);
    leg_inverse_kinematics_(msg->LEFT_LEG_CONSTANT, foot_positions_[6], foot_positions_[7], foot_positions_[8]);
    leg_inverse_kinematics_(msg->RIGHT_LEG_CONSTANT, foot_positions_[9], foot_positions_[10], foot_positions_[11]);

    // Publish joint states
    if (environment_ == "sim") {
        sim_position_controller_msg_.data = joint_angles_;
        sim_position_controller_pub_->publish(sim_position_controller_msg_);
    }
    else if (environment_ == "real") {
        joint_state_msg_.header.stamp = msg->header.stamp;
        joint_state_msg_.position = joint_angles_;

        joint_state_pub_->publish(joint_state_msg_);
    }
}

InverseKinematicsLifecycleNode::InverseKinematicsLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("InverseKinematicsLifecycleNode", options)
{
    RCLCPP_INFO(get_logger(), "InverseKinematicsLifecycleNode constructed (unconfigured).");
}

CallbackReturn InverseKinematicsLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    joint_state_msg_ = sensor_msgs::msg::JointState();
    joint_state_msg_.name = ik_config_.joint_names;
    sim_position_controller_msg_ = std_msgs::msg::Float64MultiArray();

    // Create publisher (Lifecycle-aware)
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::QoS(10));
    sim_position_controller_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/snoopy_position_controller/commands", rclcpp::QoS(10));
    // Create subscriber
    leg_position_sub_ = this->create_subscription<comm_utils::msg::LegPosition>(
        "leg_position_cmd", 10,
        std::bind(&InverseKinematicsLifecycleNode::leg_position_callback_, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(get_logger(), "Configuring inverse kinemeatics node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn InverseKinematicsLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
    joint_state_pub_->on_activate();
    sim_position_controller_pub_->on_activate();
    
    RCLCPP_INFO(get_logger(), "Activating inverse kinemeatics node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn InverseKinematicsLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    joint_state_pub_->on_deactivate();
    sim_position_controller_pub_->on_deactivate();
    
    RCLCPP_INFO(get_logger(), "Deactivating inverse kinemeatics node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn InverseKinematicsLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    joint_state_pub_.reset();
    sim_position_controller_pub_.reset();
    leg_position_sub_.reset();
    
    RCLCPP_INFO(get_logger(), "Cleaning up inverse kinemeatics  node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn InverseKinematicsLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down from state: %s", state.label().c_str());

    return CallbackReturn::SUCCESS;
}

// Register this component so it can be loaded into a component container
RCLCPP_COMPONENTS_REGISTER_NODE(InverseKinematicsLifecycleNode)
