#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "comm_utils/msg/leg_position.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <cmath>
#include <vector>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

struct IK_Config
{   
    // Joint names
    const std::vector<std::string> joint_names = {
        "LF_HIP_JOINT",
        "LF_THIGH_JOINT",
        "LF_THIGH_FOOT_JOINT",
        "RF_HIP_JOINT",
        "RF_THIGH_JOINT",
        "RF_THIGH_FOOT_JOINT",
        "LH_HIP_JOINT",
        "LH_THIGH_JOINT",
        "LH_THIGH_FOOT_JOINT",
        "RH_HIP_JOINT",
        "RH_THIGH_JOINT",
        "RH_THIGH_FOOT_JOINT"
    };
    
    // Servo angle offsets for different environments
    const double sim_theta_2_offset = -0.1784;
    const double sim_theta_3_offset = 0.2617;
    const double real_theta_2_offset = 0.0;      // TODO: find real offsets
    const double real_theta_3_offset = 0.0;;     // TODO: find real offsets
    std::map<std::pair<std::string, std::string>, double> servo_angle_offsets;

    // Offsets from hip to virtual hip joint
    const double hip_offset_x = 0.02735;
    const double hip_offset_z = 0.0012724;

    // Length of hip, upper leg, lower leg segments
    const double h = 0.029523;
    const double hu = sqrt(pow(0.022028, 2) + pow(0.12152, 2) + pow(0.0023333, 2));
    const double hl = sqrt(pow(0.01, 2) + pow(0.116, 2));

    IK_Config()
    {
        servo_angle_offsets[{"sim", "theta_2"}] = sim_theta_2_offset;
        servo_angle_offsets[{"sim", "theta_3"}] = sim_theta_3_offset;
        servo_angle_offsets[{"real", "theta_2"}] = real_theta_2_offset;
        servo_angle_offsets[{"real", "theta_3"}] = real_theta_3_offset;
    }
};

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
        IK_Config ik_config_;
        std::string environment_; // "sim" or "real"
        std::array<double, 12> foot_positions_;
        std::vector<double> joint_angles_;

        // Functions
        void leg_inverse_kinematics_(int leg_side_constant, double x_pos, double y_pos, double z_pos);
        void leg_position_callback_(const comm_utils::msg::LegPosition::SharedPtr msg);

        // ROS elements
        sensor_msgs::msg::JointState joint_state_msg_;
        std_msgs::msg::Float64MultiArray sim_position_controller_msg_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_position_controller_pub_;
        rclcpp::Subscription<comm_utils::msg::LegPosition>::SharedPtr leg_position_sub_;
};