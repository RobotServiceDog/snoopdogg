#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/float64.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace servo_control
{
    class ServoControlLifecycleNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit ServoControlLifecycleNode(const rclcpp::NodeOptions &options);

    protected:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:
        void load_params();
        void update_servo();

        rclcpp::TimerBase::SharedPtr timer_;

        // Paramaters
        std::vector<int64_t> pins_;
        std::vector<int64_t> neutral_angles_;
        std::vector<int64_t> servo_multipliers_;

        int64_t min_pwm_;
        int64_t mid_pwm_;
        int64_t max_pwm_;
    };

} // namespace servo_control