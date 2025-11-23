#include "inverse_kinematics/inverse_kinematics.hpp"

InverseKinematics::InverseKinematics() = default;

void InverseKinematics::leg_inverse_kinematics(std::string env, std::vector<double>& joint_angles, int leg_side_constant, double x_pos, double y_pos, double z_pos)
{
    // Write xyz wrt virtual hip joint
    x_pos = x_pos - ik_config_.hip_offset_x;
    z_pos = z_pos - ik_config_.hip_offset_z;
    
    // Find theta1
    dyz_ = sqrt(pow(y_pos, 2) + pow(z_pos, 2));
    lyz_ = sqrt(pow(dyz_, 2) - pow(ik_config_.h, 2));
    gamma_yz_ = -atan(y_pos/z_pos);
    gamma_h_ = -atan(ik_config_.h/lyz_);
    // leg_side_constant is +1 for left leg, -1 for right leg
    theta1_ = gamma_yz_ + leg_side_constant * gamma_h_;
    joint_angles.push_back(theta1_);

    // Calculate variables for theta2 and theta3
    lxz_ = sqrt(pow(x_pos, 2) + pow(lyz_, 2));
    n_ = (pow(lxz_, 2) - pow(ik_config_.hu, 2) - pow(ik_config_.hl, 2)) / (2 * ik_config_.hu);
    alpha_xz_ = -atan(x_pos/lyz_);
    arg_alpha_off_ = std::clamp((ik_config_.hu + n_) / lxz_, -1.0 + 1e-5, 1.0 - 1e-5);
    alpha_off_ = acos(arg_alpha_off_);

    // Find theta2
    theta2_ = alpha_xz_ + alpha_off_;

    // Find theta3
    arg_theta3_ = std::clamp(n_ / ik_config_.hl, -1.0 + 1e-5, 1.0 - 1e-5);
    theta3_ = -acos(arg_theta3_);

    if (env == "sim") {
        theta2_ -= ik_config_.servo_angle_offsets[{env, "theta_2"}];
        theta3_ -= ik_config_.servo_angle_offsets[{env, "theta_3"}];
    }
    else if (env == "real") {
        theta3_ = ik_config_.servo_angle_offsets[{env, "theta_3"}] + std::abs(theta2_) - std::abs(theta3_);
        theta2_ -= ik_config_.servo_angle_offsets[{env, "theta_2"}];
    }

    joint_angles.push_back(theta2_);
    joint_angles.push_back(theta3_);
};