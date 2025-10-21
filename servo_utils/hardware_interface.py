from numpy import pi
import pigpio

from servo_utils.config import (
    PWMParams, ServoParams, NUM_AXES, NUM_LEGS
)

class HardwareInterface:
    def __init__(self):
        self.pi = pigpio.pi()
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.initialize_pwm(self.pi, self.pwm_params)

    def set_actuator_positions(self, joint_angles):
        """Sets the positions of all actuators based on the provided joint angles."""
        for axis in range(NUM_AXES):
            for leg in range(NUM_LEGS):
                angle = joint_angles[axis][leg]
                self.send_servo_command(angle, axis, leg)

    def set_actuator_position(self, joint_angle, axis, leg):
        """Sets the position of a single actuator."""
        self.send_servo_command(joint_angle, axis, leg)

    def send_servo_command(self, angle, axis, leg):
        """Converts a desired angle to PWM and sends the command to the servo."""
        pwm_value = self.angle_to_pwm(angle, axis, leg)
        
        # Verify PWM is within bounds
        if not (self.servo_params.min_pwm <= pwm_value <= self.servo_params.max_pwm):
            raise ValueError(
                f"Angle {angle} corresponding to PWM value {pwm_value} for axis {axis}, leg {leg} is out of bounds."
            )
        
        self.pi.set_servo_pulsewidth(self.pwm_params.pins[axis][leg], pwm_value)

    def angle_to_pwm(self, angle, axis, leg):
        """Converts a joint angle (in radians) to a PWM pulse width (in microseconds)."""
        neutral_angle = self.servo_params.neutral_angles[axis][leg]
        neutral_pwm = self.servo_params.mid_pwm
        micros_per_rad = self.servo_params.micros_per_rad
        multiplier = self.servo_params.servo_multipliers[axis][leg]

        pwm_value = int(
            neutral_pwm + multiplier * micros_per_rad * (angle - neutral_angle)
        )
        return pwm_value

    def initialize_pwm(self, pi, pwm_params):
        """Initializes PWM frequency for all servo pins."""
        for axis_index in range(NUM_AXES):
            for leg_index in range(NUM_LEGS):
                pi.set_PWM_frequency(
                    pwm_params.pins[axis_index, leg_index], pwm_params.freq
                )
        print(f"PWM Frequencies set to {pwm_params.freq} Hz for all servos.")

    def stop_all(self):
        """Stops all servos by setting their pulsewidth to 0."""
        for axis in range(NUM_AXES):
            for leg in range(NUM_LEGS):
                self.pi.set_servo_pulsewidth(self.pwm_params.pins[axis][leg], 0)

    def close(self):
        """Cleans up the pigpio interface."""
        self.stop_all()
        self.pi.stop()