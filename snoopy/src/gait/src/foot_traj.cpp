#include "gait/foot_traj.hpp"

FootTrajectory::FootTrajectory()
    : swing_height_(0.05)  // default 5 cm clearance
{}

void FootTrajectory::setSwingParams(double swing_height)
{
    swing_height_ = swing_height;
}

Eigen::Vector3d FootTrajectory::getTrajectory(
    double phase,
    double duty_factor,
    double stride_time,
    Eigen::Vector3d neutral_foot_pos,
    double height,
    double velocity)
{
    double p = std::min(std::max(phase, 0.0), 1.0);
    double displacement = velocity * stride_time;

    Eigen::Vector3d pos;
    Eigen::Vector3d start_pos = neutral_foot_pos;
    Eigen::Vector3d end_pos = neutral_foot_pos;
    start_pos.x() = start_pos.x() + displacement/2.0 + 0.01 * velocity;
    end_pos.x() = end_pos.x() - displacement/2.0 + 0.01 * velocity;

    if (p <= duty_factor) {
        double s = p / duty_factor;
        pos = start_pos;
        pos.x() -= (s * displacement);
    }
    else {
        double s = (p - duty_factor) / (1 - duty_factor);
        pos = end_pos;
        pos.x() += (s * displacement);
        double h = 4 * height * s * (1 - s);
        pos.z() += h;
    }

   return pos;
}
