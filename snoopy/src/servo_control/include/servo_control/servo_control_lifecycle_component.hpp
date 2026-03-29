
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "servo_control/hardware_interface.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
        void init_subscribers();
        void timer_callback();
        void update_servo();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
        sensor_msgs::msg::JointState current_joint_state_;

        // Hardware Interface
        std::unique_ptr<HardwareInterface> hardware_interface_;
    };

} // namespace servo_control