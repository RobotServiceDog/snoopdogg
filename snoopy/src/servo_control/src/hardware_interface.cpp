
#include "servo_control/hardware_interface.hpp"

HardwareInterface::HardwareInterface()
{
    pi_handle = pigpio_start(NULL, NULL);
    if (pi_handle < 0)
        throw std::runtime_error("Failed to connect to pigpiod daemon!");

    pwm_params = PWMParams();
    servo_params = ServoParams();

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
            set_PWM_frequency(pi_handle, pwm_params.pins[axis][leg], pwm_params.freq);
        }
    }

    std::cout << "PWM frequencies initialized." << std::endl;
}

int HardwareInterface::angle_to_pwm(double angle, int axis, int leg)
{
    double neutral = servo_params.neutral_angles[axis][leg];
    int mid = servo_params.mid_pwm;
    double micros_per_rad = servo_params.micros_per_rad;
    double mult = servo_params.servo_multipliers[axis][leg];

    return static_cast<int>(mid + mult * micros_per_rad * (angle - neutral));
}

void HardwareInterface::send_servo_command(double angle, int axis, int leg)
{
    int pwm_value = angle_to_pwm(angle, axis, leg);

    if (pwm_value < servo_params.min_pwm || pwm_value > servo_params.max_pwm)
    {
        throw std::runtime_error(
            "PWM out of bounds: axis=" + std::to_string(axis) +
            ", leg=" + std::to_string(leg) +
            ", pwm=" + std::to_string(pwm_value));
    }

    set_servo_pulsewidth(pi_handle, pwm_params.pins[axis][leg], pwm_value);
}

void HardwareInterface::set_actuator_positions(const std::vector<double> &angles)
{
    if (angles.size() != NUM_AXES * NUM_LEGS)
        throw std::runtime_error("Invalid angle vector size!");

    for (int axis = 0; axis < NUM_AXES; axis++)
    {
        for (int leg = 0; leg < NUM_LEGS; leg++)
        {
            double angle = angles[axis * NUM_LEGS + leg];
            send_servo_command(angle, axis, leg);
        }
    }
}

void HardwareInterface::set_actuator_position(double angle, int axis, int leg)
{
    send_servo_command(angle, axis, leg);
}

void HardwareInterface::stop_all()
{
    for (int axis = 0; axis < NUM_AXES; axis++)
    {
        for (int leg = 0; leg < NUM_LEGS; leg++)
        {
            set_servo_pulsewidth(pi_handle, pwm_params.pins[axis][leg], 0);
        }
    }
}
