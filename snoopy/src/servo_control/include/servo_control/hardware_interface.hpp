
#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>
#include <pigpiod_if2.h>

// Replace with your actual config values / include files

class HardwareInterface
{
public:
    HardwareInterface();
    ~HardwareInterface();

    void set_actuator_positions(const std::vector<double> &angles);
    void set_actuator_position(double angle, int axis, int leg);
    void stop_all();

private:
    int pi_handle;

    PWMParams pwm_params;
    ServoParams servo_params;

    void initialize_pwm();
    int angle_to_pwm(double angle, int axis, int leg);
    void send_servo_command(double angle, int axis, int leg);
};
