
#include "servo_control/hardware_interface.hpp"

namespace servo_control
{

    HardwareInterface::HardwareInterface()
    {
    }

    HardwareInterface::~HardwareInterface()
    {
    }

    void HardwareInterface::initialize_pwm()
    {
        for (int axis = 0; axis < NUM_AXES; axis++)
        {
            for (int leg = 0; leg < NUM_LEGS; leg++)
            {
                // set_PWM_frequency(pi_handle, pwm_params.pins[axis][leg], pwm_params.freq);
            }
        }

        std::cout << "PWM frequencies initialized." << std::endl;
    }

    int HardwareInterface::angle_to_pwm(double angle, int axis, int leg)
    {
        return 1;
    }

    void HardwareInterface::send_servo_command(double angle, int axis, int leg)
    {
    }

    void HardwareInterface::set_actuator_positions(const std::vector<double> &angles)
    {
    }

    void HardwareInterface::set_actuator_position(double angle, int axis, int leg)
    {
    }

    void HardwareInterface::stop_all()
    {
        // for (int axis = 0; axis < NUM_AXES; axis++)
        // {
        //     for (int leg = 0; leg < NUM_LEGS; leg++)
        //     {
        //         // set_servo_pulsewidth(pi_handle, pwm_params.pins[axis][leg], 0);
        //     }
        // }
    }

} // namespace servo_control