
#include "servo_control/servo_control_lifecycle_component.hpp"

namespace servo_control
{
    ServoControlLifecycleNode::ServoControlLifecycleNode(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("ServoControlLifecycleNode", options)
    {
        RCLCPP_INFO(get_logger(), "ServoControlLifecycleNode constructed (unconfigured).");
    }

    CallbackReturn ServoControlLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring servo...");

        this->init_subscribers();

        // try
        // {
        //     hardware_interface_ = std::make_unique<HardwareInterface>();
        // }
        // catch (const std::exception &e)
        // {
        //     RCLCPP_ERROR(get_logger(), "HardwareInterface init failed: %s", e.what());
        //     return CallbackReturn::FAILURE;
        // }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ServoControlLifecycleNode::timer_callback, this));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ServoControlLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating servo...");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ServoControlLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating servo...");

        timer_->cancel();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ServoControlLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up servo node...");

        timer_.reset();
        hardware_interface_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ServoControlLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Shutting down from state: %s", state.label().c_str());

        hardware_interface_.reset();
        return CallbackReturn::SUCCESS;
    }

    void ServoControlLifecycleNode::init_subscribers()
    {
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                current_joint_state_ = *msg;
                RCLCPP_INFO(get_logger(), "Received joint state message with %zu joints", current_joint_state_.name.size());
            });
    }


    void ServoControlLifecycleNode::timer_callback()
    {
        RCLCPP_INFO(get_logger(), "Timer callback triggered.");
        // if (!hardware_interface_)
        //     return;

        // try
        // {
        //     hardware_interface_->set_actuator_positions(current_joint_state_.position);
        // }
        // catch (const std::exception &e)
        // {
        //     RCLCPP_ERROR(get_logger(), "Servo command failed: %s", e.what());
        // }
    }

} // namespace servo_control

RCLCPP_COMPONENTS_REGISTER_NODE(servo_control::ServoControlLifecycleNode)