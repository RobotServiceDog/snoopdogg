
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

        this->load_params();

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

    void ServoControlLifecycleNode::load_params()
    {
        // --- PWMs ---
        this->declare_parameter("min_pwm", 680);
        this->declare_parameter("mid_pwm", 1500);
        this->declare_parameter("max_pwm", 232);

        this->get_parameter("min_pwm", min_pwm_);
        this->get_parameter("mid_pwm", mid_pwm_);
        this->get_parameter("max_pwm", max_pwm_);

        // --- Pins ---
        this->declare_parameter("pins", std::vector<int>{});
        this->declare_parameter("neutral_angles", std::vector<int>{});
        this->declare_parameter("servo_multipliers", std::vector<int>{});

        this->get_parameter("pins", pins_);
        this->get_parameter("neutral_angles", neutral_angles_);
        this->get_parameter("servo_multipliers", servo_multipliers_);

        // --- Logging for verification ---
        RCLCPP_INFO(get_logger(), "Loaded scalar params: min_pwm=%d, mid_pwm=%d, max_pwm=%d",
                    min_pwm_, mid_pwm_, max_pwm_);
    }

    void ServoControlLifecycleNode::update_servo()
    {
    }

} // namespace servo_control

// Register this component so it can be loaded into a component container
RCLCPP_COMPONENTS_REGISTER_NODE(servo_control::ServoControlLifecycleNode)