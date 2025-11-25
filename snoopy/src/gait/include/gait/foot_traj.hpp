#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>

class FootTrajectory {
public:
    FootTrajectory();

    // Set swing parameters
    void setSwingParams(double swing_height);

    Eigen::Vector3d getTrajectory(
        double phase,
        double duty_factor,
        double stride_time,
        Eigen::Vector3d neutral_foot_pos,
        double height,
        double velocity);

private:
    double swing_height_;
};
