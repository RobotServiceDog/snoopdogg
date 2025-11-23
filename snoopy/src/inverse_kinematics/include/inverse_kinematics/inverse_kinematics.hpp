#pragma once

#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <algorithm>

struct IK_Config
{   
    // Joint names
    std::vector<std::string> joint_names;
    
    // Servo angle offsets for different environments
    double sim_theta_2_offset;
    double sim_theta_3_offset;
    double real_theta_2_offset;      
    double real_theta_3_offset;     
    std::map<std::pair<std::string, std::string>, double> servo_angle_offsets;

    // Offsets from hip to virtual hip joint
    double hip_offset_x;
    double hip_offset_z;

    // Length of hip, upper leg, lower leg segments
    double h;
    double hu;
    double hl;
};

class InverseKinematics
{
    public:
        InverseKinematics();

        IK_Config ik_config_;
        void leg_inverse_kinematics(std::string env, std::vector<double>& joint_angles, int leg_side_constant, double x_pos, double y_pos, double z_pos);
    
    private:
        // Helper variables for inverse kinematics calculations
        double dyz_;
        double lyz_;
        double gamma_yz_;
        double gamma_h_;
        double theta1_;
        double lxz_;
        double n_;
        double alpha_xz_;
        double arg_alpha_off_;
        double alpha_off_;
        double theta2_;
        double arg_theta3_;
        double theta3_;

};