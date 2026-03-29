
#include "servo_control/hardware_interface.hpp"

namespace servo_control
{

    HardwareInterface::HardwareInterface()
    {
          // Initialize pigpio library
            pi_handle = pigpio_start(NULL, NULL);
            if (pi_handle < 0)
            {
                throw std::runtime_error("Failed to connect to pigpio daemon");
            }

        std::cout << "Pigpio initialized successfully." << std::endl;

        // Initialize your PWM and servo parameters
        initialize_pwm();
    }

    HardwareInterface::~HardwareInterface()
    {
        stop_all();
        pigpio_stop(pi_handle);
    }

    void HardwareInterface::initialize_pwm()
    {
        for (int axis = 0; axis < NUM_AXES; axis++)
        {
            for (int leg = 0; leg < NUM_LEGS; leg++)
            {
                int pin = pwm_params_.pins[leg][axis];  
                // No frequency set in pigpiod_if2; just ensure initial pulse width
                set_servo_pulsewidth(pi_handle, pin, 0);     
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
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            for (int axis = 0; axis < NUM_AXES; ++axis)
            {
                int pin = pwm_params_.pins[leg][axis];
                set_servo_pulsewidth(pi_handle, pin, 0); // stop servo
            }
        }
        std::cout << "All servos stopped." << std::endl;
    }

} // namespace servo_control