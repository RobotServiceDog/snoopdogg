#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "comm_utils/msg/leg_position.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "inverse_kinematics/leg_ik.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class InverseKinematicsLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit InverseKinematicsLifecycleNode(const rclcpp::NodeOptions &options);

    protected:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:
        InverseKinematics ik_solver_;

        std::string environment_; // "sim" or "real"
        std::array<double, 12> foot_positions_;
        std::vector<double> joint_angles_;

        void load_parameters_();

        // Functions
        void leg_position_callback_(const comm_utils::msg::LegPosition::SharedPtr msg);

        // ROS elements
        sensor_msgs::msg::JointState joint_state_msg_;
        std_msgs::msg::Float64MultiArray sim_position_controller_msg_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_position_controller_pub_;
        rclcpp::Subscription<comm_utils::msg::LegPosition>::SharedPtr leg_position_sub_;
};