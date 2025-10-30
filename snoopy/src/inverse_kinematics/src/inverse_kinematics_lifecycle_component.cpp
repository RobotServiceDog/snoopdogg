
#include "inverse_kinematics/inverse_kinematics_lifecycle_component.hpp"

InverseKinematicsLifecycleNode::InverseKinematicsLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("InverseKinematicsLifecycleNode", options)
{
    RCLCPP_INFO(get_logger(), "InverseKinematicsLifecycleNode constructed (unconfigured).");
}

CallbackReturn InverseKinematicsLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring inverse kinemeatics node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn InverseKinematicsLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating inverse kinemeatics node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn InverseKinematicsLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating inverse kinemeatics node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn InverseKinematicsLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
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
