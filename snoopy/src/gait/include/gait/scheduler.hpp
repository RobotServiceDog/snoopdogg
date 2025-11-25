#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

class GaitScheduler {
public:
    GaitScheduler();

    void setGaitParams(double stride_time,
                       double duty_factor,
                       const std::vector<double>& phase_offsets);

    void update(double dt,
                const Eigen::Vector3d& v_des,
                const Eigen::Vector3d& v_actual,
                const std::vector<Eigen::Vector3d>& foot_positions_body);

    struct LegState {
        bool in_stance;
        double phase;         
    };

    const std::vector<LegState>& getLegStates() const { return leg_states_; }

    void setInitialLegStates(const std::vector<LegState>& initial_states) {
        leg_states_ = initial_states;
    }

private:
    double stride_time_;    // total cycle time (stance + swing)
    double duty_factor_;    // stance ratio (stance_time/stride_time)
    
    std::vector<double> phase_offsets_;
    double global_phase_;   // updated continuously

    std::vector<LegState> leg_states_;

    // Raibert gains
    double k_foot_placement_;
};
