
#include "inverse_kinematics/inverse_kinematics_lifecycle_component.hpp"

void InverseKinematicsLifecycleNode::load_parameters_()
{
    this->get_parameter("joint_names", ik_solver_.ik_config_.joint_names);
    this->get_parameter("sim_theta_2_offset", ik_solver_.ik_config_.sim_theta_2_offset);
    this->get_parameter("sim_theta_3_offset", ik_solver_.ik_config_.sim_theta_3_offset);
    this->get_parameter("real_theta_2_offset", ik_solver_.ik_config_.real_theta_2_offset);
    this->get_parameter("real_theta_3_offset", ik_solver_.ik_config_.real_theta_3_offset);
    this->get_parameter("hip_offset_x", ik_solver_.ik_config_.hip_offset_x);
    this->get_parameter("hip_offset_z", ik_solver_.ik_config_.hip_offset_z);
    this->get_parameter("h", ik_solver_.ik_config_.h);
    this->get_parameter("hu", ik_solver_.ik_config_.hu);
    this->get_parameter("hl", ik_solver_.ik_config_.hl);
}

void InverseKinematicsLifecycleNode::leg_position_callback_(const comm_utils::msg::LegPosition::SharedPtr msg)
{
    // Clear previous data
    joint_angles_.clear();

    // Process incoming message
    environment_ = msg->env;
    foot_positions_ = msg->foot_position;

    ik_solver_.leg_inverse_kinematics(environment_, joint_angles_, msg->LEFT_LEG_CONSTANT, foot_positions_[0], foot_positions_[1], foot_positions_[2]);
    RCLCPP_INFO(this->get_logger(), "Joint angles calculated: [theta1: %f, theta2: %f, theta3: %f]", joint_angles_[0], joint_angles_[1], joint_angles_[2]);
    ik_solver_.leg_inverse_kinematics(environment_, joint_angles_, msg->RIGHT_LEG_CONSTANT, foot_positions_[3], foot_positions_[4], foot_positions_[5]);
    RCLCPP_INFO(this->get_logger(), "Joint angles calculated: [theta1: %f, theta2: %f, theta3: %f]", joint_angles_[3], joint_angles_[4], joint_angles_[5]);
    ik_solver_.leg_inverse_kinematics(environment_, joint_angles_, msg->LEFT_LEG_CONSTANT, foot_positions_[6], foot_positions_[7], foot_positions_[8]);
    RCLCPP_INFO(this->get_logger(), "Joint angles calculated: [theta1: %f, theta2: %f, theta3: %f]", joint_angles_[6], joint_angles_[7], joint_angles_[8]);
    ik_solver_.leg_inverse_kinematics(environment_, joint_angles_, msg->RIGHT_LEG_CONSTANT, foot_positions_[9], foot_positions_[10], foot_positions_[11]);
    RCLCPP_INFO(this->get_logger(), "Joint angles calculated: [theta1: %f, theta2: %f, theta3: %f]", joint_angles_[9], joint_angles_[10], joint_angles_[11]);

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

    this->declare_parameter<std::vector<std::string>>("joint_names");
    this->declare_parameter<double>("sim_theta_2_offset");
    this->declare_parameter<double>("sim_theta_3_offset");
    this->declare_parameter<double>("real_theta_2_offset");
    this->declare_parameter<double>("real_theta_3_offset");
    this->declare_parameter<double>("hip_offset_x");
    this->declare_parameter<double>("hip_offset_z");
    this->declare_parameter<double>("h");
    this->declare_parameter<double>("hu");
    this->declare_parameter<double>("hl");

}

CallbackReturn InverseKinematicsLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{   
    ik_solver_ = InverseKinematics();

    load_parameters_();
    ik_solver_.ik_config_.servo_angle_offsets[{"sim", "theta_2"}] = ik_solver_.ik_config_.sim_theta_2_offset;
    ik_solver_.ik_config_.servo_angle_offsets[{"sim", "theta_3"}] = ik_solver_.ik_config_.sim_theta_3_offset;
    ik_solver_.ik_config_.servo_angle_offsets[{"real", "theta_2"}] = ik_solver_.ik_config_.real_theta_2_offset;
    ik_solver_.ik_config_.servo_angle_offsets[{"real", "theta_3"}] = ik_solver_.ik_config_.real_theta_3_offset;

    joint_state_msg_ = sensor_msgs::msg::JointState();
    joint_state_msg_.name = ik_solver_.ik_config_.joint_names;
    sim_position_controller_msg_ = std_msgs::msg::Float64MultiArray();

    // Create publisher (Lifecycle-aware)
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::QoS(10));
    sim_position_controller_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/snoopy_position_controller/commands", rclcpp::QoS(10));
    // Create subscriber
    leg_position_sub_ = this->create_subscription<comm_utils::msg::LegPosition>(
        "/leg_position_cmd", 10,
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
