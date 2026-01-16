#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "comm_utils/msg/leg_position.hpp"
#include <geometry_msgs/msg/vector3.hpp>

#include "gait/scheduler.hpp"
#include "gait/foot_traj.hpp"

#include <eigen3/Eigen/Dense>
#include <vector>
#include <chrono>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GaitLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit GaitLifecycleNode(const rclcpp::NodeOptions &options);

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
    void load_parameters_();
    void controlLoop_();

    // ROS interfaces
    rclcpp::TimerBase::SharedPtr timer_;

    // Gait components
    GaitScheduler scheduler_;
    FootTrajectory foot_traj_;

    // State
    Eigen::Vector3d v_des_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_actual_ = Eigen::Vector3d::Zero(); // we will set = v_des to remove feedback
    std::vector<Eigen::Vector3d> foot_positions_body_; // current/last foot positions (body frame)

    // Params
    double stride_time_;
    double duty_factor_;
    double swing_height_;
    double control_rate_hz_;
    double k_foot_placement_; // mirrored to scheduler if needed

    // neutral footholds (4 legs: LF, RF, LH, RH)
    std::vector<Eigen::Vector3d> neutral_foot_positions_;

    void v_des_callback_(const geometry_msgs::msg::Vector3::SharedPtr msg);

    rclcpp_lifecycle::LifecyclePublisher<comm_utils::msg::LegPosition>::SharedPtr leg_position_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr v_des_sub_;
};
