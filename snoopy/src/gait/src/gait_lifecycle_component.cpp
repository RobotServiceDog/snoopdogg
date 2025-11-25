#include "gait/gait_lifecycle_component.hpp"

GaitLifecycleNode::GaitLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("gait_lifecycle_node", options)
{
    RCLCPP_INFO(get_logger(), "GaitLifecycleNode constructed (unconfigured).");
}

void GaitLifecycleNode::load_parameters_()
{
    // Gait params
    stride_time_  = this->declare_parameter<double>("stride_time", 2.0);
    duty_factor_  = this->declare_parameter<double>("duty_factor", 0.8);
    swing_height_ = this->declare_parameter<double>("swing_height", 0.03);
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 10.0);

    // Optional foot placement gain (kept in scheduler too)
    k_foot_placement_ = this->declare_parameter<double>("k_foot_placement", 0.05);

    // Neutral foot positions (12 values: LF xyz, RF xyz, LH xyz, RH xyz)
    // default: LF(+x,+y), RF(+x,-y), LH(-x,+y), RH(-x,-y)
    std::vector<double> neutral_list = this->declare_parameter<std::vector<double>>(
        "neutral_foot_positions",
        std::vector<double>{
            0.032,  0.032, -0.15,   // LF
            0.032, -0.032, -0.15,   // RF
            0.032,  0.032, -0.15,   // LH
            0.032, -0.032, -0.15    // RH
        });

    neutral_foot_positions_.clear();
    neutral_foot_positions_.reserve(4);
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d p(neutral_list[3*i + 0], neutral_list[3*i + 1], neutral_list[3*i + 2]);
        neutral_foot_positions_.push_back(p);
    }

    // Initialize foot positions (start at neutral)
    foot_positions_body_ = neutral_foot_positions_;

    // Configure scheduler params
    scheduler_.setGaitParams(stride_time_, duty_factor_, std::vector<double>{0.0, 0.5, 0.5, 0.0});
    // ensure scheduler internal k is same as parameter (optional, not exposed in your class)
    // scheduler_.setFootPlacementGain(k_foot_placement_); // if you add this setter later

    // Configure foot trajectory
    foot_traj_.setSwingParams(swing_height_);

    RCLCPP_INFO(get_logger(), "Parameters loaded: stride_time=%.3f duty=%.3f swing_h=%.3f rate=%.1f",
                stride_time_, duty_factor_, swing_height_, control_rate_hz_);

    v_des_ = {0.1, 0.0, 0.0}; // default desired velocity: 0.1 m/s forward
}

void GaitLifecycleNode::controlLoop_()
{
    // compute dt
    double dt = 1.0 / control_rate_hz_;

    // We do NOT have feedback v_actual; to disable feedback correction in your scheduler,
    // pass v_actual = v_des (so vel_error = 0). This keeps the scheduler feedforward.
    v_actual_ = v_des_;

    // Update scheduler
    // The scheduler expects foot_positions_body_ representing current footholds in body frame.
    scheduler_.update(dt, v_des_, v_actual_, foot_positions_body_);
    const auto &leg_states = scheduler_.getLegStates();

    double stance_time = stride_time_ * duty_factor_;

    Eigen::Vector3d new_foot_pos;
    
    // Update foot trajectories
    for (size_t i = 0; i < 4; ++i) {
        const auto &leg = leg_states[i];
        Eigen::Vector3d new_foot_pos;

        new_foot_pos = foot_traj_.getTrajectory(
            leg.phase,
            duty_factor_,
            stride_time_,
            neutral_foot_positions_[i],
            swing_height_,
            v_des_[0]);
    
        foot_positions_body_[i] = new_foot_pos;
        RCLCPP_INFO(get_logger(),
            "  Foot pos (body frame): x=%.3f y=%.3f z=%.3f",
            new_foot_pos.x(), new_foot_pos.y(), new_foot_pos.z());
    }

    comm_utils::msg::LegPosition msg;
    msg.header.stamp = now();
    msg.env = comm_utils::msg::LegPosition::ENV_SIM; // or ENV_SIM

    // Fill the 12-element array
    for (int i = 0; i < 4; ++i) {
        msg.foot_position[3*i + 0] = foot_positions_body_[i].x();
        msg.foot_position[3*i + 1] = foot_positions_body_[i].y();
        msg.foot_position[3*i + 2] = foot_positions_body_[i].z();
    }

    leg_position_pub_->publish(msg);


}

CallbackReturn GaitLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring...");

    load_parameters_();

    leg_position_pub_ = this->create_publisher<comm_utils::msg::LegPosition>("leg_position_cmd", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "Configured.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn GaitLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating...");

    leg_position_pub_->on_activate();

    std::vector<GaitScheduler::LegState> initial_states(4);
    for (int i = 0; i < 4; ++i) {
        initial_states[i].in_stance = true;
        initial_states[i].phase = 0.0;
    }
    scheduler_.setInitialLegStates(initial_states);

    // start control loop timer
    double dt = 1.0 / control_rate_hz_;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&GaitLifecycleNode::controlLoop_, this));

    RCLCPP_INFO(get_logger(), "Activated.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn GaitLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating...");
    leg_position_pub_->on_deactivate();
    timer_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn GaitLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    timer_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn GaitLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down...");
    return CallbackReturn::SUCCESS;
}

RCLCPP_COMPONENTS_REGISTER_NODE(GaitLifecycleNode)
