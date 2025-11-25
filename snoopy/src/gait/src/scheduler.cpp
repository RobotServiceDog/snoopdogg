#include "gait/scheduler.hpp"
#include <iostream>

GaitScheduler::GaitScheduler()
    : stride_time_(1.0),
      duty_factor_(0.6),
      global_phase_(0.0),
      k_foot_placement_(0.05)
{
    leg_states_.resize(4);
    phase_offsets_ = {0.0, 0.5, 0.5, 0.0}; // LF RF LH RH
}

void GaitScheduler::setGaitParams(double stride_time,
                                  double duty_factor,
                                  const std::vector<double>& phase_offsets)
{
    stride_time_ = stride_time;
    duty_factor_ = duty_factor;
    phase_offsets_ = phase_offsets;
}

void GaitScheduler::update(double dt,
                           const Eigen::Vector3d& v_des,
                           const Eigen::Vector3d& v_actual,
                           const std::vector<Eigen::Vector3d>& foot_positions_body)
{
    // -----------------------------------------------------
    // 1. Update global phase (looping 0–1)
    // -----------------------------------------------------
    global_phase_ += dt / stride_time_;
    if (global_phase_ >= 1.0) global_phase_ -= 1.0;

    for (int i = 0; i < 4; ++i) {
        LegState& ls = leg_states_[i];

        // ---------------------------------------------
        // 2. Compute leg-specific phase
        // ---------------------------------------------
        double phase = fmod(global_phase_ + phase_offsets_[i], 1.0);
        if (phase < 0.0) phase += 1.0;
        ls.phase = phase;

        if (ls.phase <= duty_factor_) {
            ls.in_stance = true;
        }
        else {
            ls.in_stance = false;
        }
    }
}
