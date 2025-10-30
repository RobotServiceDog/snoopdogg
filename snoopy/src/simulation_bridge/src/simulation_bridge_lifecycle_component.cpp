
#include "simulation_bridge/simulation_bridge_lifecycle_component.hpp"

SimulationBridgeLifecycleNode::SimulationBridgeLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("SimulationBridgeLifecycleNode", options)
{
    RCLCPP_INFO(get_logger(), "SimulationBridgeLifecycleNode constructed (unconfigured).");
}

CallbackReturn SimulationBridgeLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring Simulation Bridge...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn SimulationBridgeLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating Simulation Bridge...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn SimulationBridgeLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating Simulation Bridge...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn SimulationBridgeLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up Simulation Bridge node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn SimulationBridgeLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down from state: %s", state.label().c_str());

    return CallbackReturn::SUCCESS;
}

// Register this component so it can be loaded into a component container
RCLCPP_COMPONENTS_REGISTER_NODE(SimulationBridgeLifecycleNode)
