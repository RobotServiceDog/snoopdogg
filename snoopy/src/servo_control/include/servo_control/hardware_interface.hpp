
#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>

#ifdef PI
#include <pigpiod_if2.h>
#endif

#define NUM_AXES 3
#define NUM_LEGS 4

namespace servo_control
{
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
        
        void initialize_pwm();
        int angle_to_pwm(double angle, int axis, int leg);
        void send_servo_command(double angle, int axis, int leg);
    };
} // namespace servo_control