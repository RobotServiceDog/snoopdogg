
#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>
#include <unordered_map>

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

        // Params
        struct PWMParams
        {
            int pins[NUM_LEGS][NUM_AXES] = {
                {2, 3, 4},   // Leg 0 pins (Axis 0, 1, 2)
                {14, 15, 17},   // Leg 1 pins (Axis 0, 1, 2)
                {18, 27, 22},   // Leg 2 pins (Axis 0, 1, 2)
                {23, 24, 25}   // Leg 3 pins (Axis 0, 1, 2)
            };
            int min_pulsewidth = 680;  // Minimum pulse width in microseconds
            int max_pulsewidth = 2320; // Maximum pulse width in microseconds

            double micros_per_rad = 11.3333 * 180.0 / 3.141592653589793; // Microseconds per radian;
            
            double neutral_angles[NUM_LEGS][NUM_AXES] = {
                {0.0, 0.0, 0.0},   // Leg 0 neutral angles (Axis 0, 1, 2)
                {0.0, 0.0, 0.0},   // Leg 1 neutral angles (Axis 0, 1, 2)
                {0.0, 0.0, 0.0},   // Leg 2 neutral angles (Axis 0, 1, 2)
                {0.0, 0.0, 0.0}    // Leg 3 neutral angles (Axis 0, 1, 2)
            };

            int serv_multipliers[NUM_LEGS][NUM_AXES] = {
                {1, 1, -1},   // Leg 0 multipliers (Axis 0, 1, 2)
                {1, 1, -1},   // Leg 1 multipliers (Axis 0, 1, 2)
                {1, 1, -1},   // Leg 2 multipliers (Axis 0, 1, 2)
                {1, 1, -1}    // Leg 3 multipliers (Axis 0, 1, 2)
            };
        
        } pwm_params;

    };
} // namespace servo_control