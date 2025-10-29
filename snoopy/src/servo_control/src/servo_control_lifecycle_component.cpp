
#include "servo_control/servo_control_lifecycle_component.hpp"

ServoControlLifecycleNode::ServoControlLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("ServoControlLifecycleNode", options)
{
    RCLCPP_INFO(get_logger(), "ServoControlLifecycleNode constructed (unconfigured).");
}

CallbackReturn ServoControlLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring servo...");

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

    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up servo node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down from state: %s", state.label().c_str());

    return CallbackReturn::SUCCESS;
}

void ServoControlLifecycleNode::update_servo()
{
}

// Register this component so it can be loaded into a component container
RCLCPP_COMPONENTS_REGISTER_NODE(ServoControlLifecycleNode)
