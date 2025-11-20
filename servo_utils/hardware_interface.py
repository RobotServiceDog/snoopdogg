from numpy import pi
import pigpio

from servo_utils.config import (
    PWMParams, ServoParams, NUM_AXES, NUM_LEGS
)
from servo_utils.servo_calibration import (
    SERVO_PWM_RANGES,
)

class HardwareInterface:
    def __init__(self):
        self.pi = pigpio.pi()
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.initialize_pwm(self.pi, self.pwm_params)

    def set_actuator_positions(self, joint_angles):
        """Sets the positions of all actuators based on the provided joint angles."""
        for leg in range(NUM_LEGS):
            for axis in range(NUM_AXES):
                angle = joint_angles[leg][axis]
                self.send_servo_command(angle, leg, axis)

    def set_actuator_position(self, joint_angle, leg, axis):
        """Sets the position of a single actuator."""
        self.send_servo_command(joint_angle, leg, axis)

    def send_servo_command(self, angle, leg, axis):
        """Converts a desired angle to PWM and sends the command to the servo."""
        pwm_value = self.angle_to_pwm(angle, leg, axis)
        
        # Check if PWM value is within calibrated bounds
        min_bound, _, max_bound = SERVO_PWM_RANGES[leg][axis]
        if pwm_value < min_bound or pwm_value > max_bound:
            raise ValueError(
                f"Calculated PWM value {pwm_value} for angle {angle} "
                f"on axis {axis}, leg {leg} is out of calibrated bounds "
                f"({min_bound}-{max_bound})."
            )
            
        # Global servo limits check
        if not (self.servo_params.min_pwm <= pwm_value <= self.servo_params.max_pwm):
            raise ValueError(
                f"Angle {angle} corresponding to PWM value {pwm_value} for axis {axis}, leg {leg} is out of bounds."
            )
        
        self.pi.set_servo_pulsewidth(self.pwm_params.pins[leg][axis], pwm_value)

    def angle_to_pwm(self, angle, leg, axis):
        """Converts a joint angle (in radians) to a PWM pulse width (in microseconds)."""
        neutral_angle = self.servo_params.neutral_angles[leg][axis]
        _, neutral_pwm, _ = SERVO_PWM_RANGES[leg][axis]
        micros_per_rad = self.servo_params.micros_per_rad
        multiplier = self.servo_params.servo_multipliers[leg][axis]

        pwm_value = int(
            neutral_pwm + multiplier * micros_per_rad * (angle - neutral_angle)
        )
        print (f"Axis {axis}, Leg {leg}: Angle {angle:.2f} rad -> PWM {pwm_value} µs")
        return pwm_value

    def initialize_pwm(self, pi, pwm_params):
        """Initializes PWM frequency for all servo pins."""
        for leg in range(NUM_LEGS):
            for axis in range(NUM_AXES):
                pi.set_PWM_frequency(
                    pwm_params.pins[leg][axis], pwm_params.freq
                )
        print(f"PWM Frequencies set to {pwm_params.freq} Hz for all servos.")

    def stop_all(self):
        """Stops all servos by setting their pulsewidth to 0."""
        for leg in range(NUM_LEGS):
            for axis in range(NUM_AXES):
                self.pi.set_servo_pulsewidth(self.pwm_params.pins[leg][axis], 0)

    def close(self):
        """Cleans up the pigpio interface."""
        self.stop_all()
        self.pi.stop()