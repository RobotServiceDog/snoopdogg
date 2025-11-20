
import numpy as np

from servo_utils.servo_calibration import (
    MICROS_PER_RAD,
    NEUTRAL_ANGLE_DEGREES,
    SERVO_MULTIPLIERS,
    ANGLE_CONSTRAINTS_DEGREES
)

NUM_AXES = 3
NUM_LEGS = 4

class PWMParams:
    def __init__(self):
        self.pins = np.array([[2, 3, 4], [14, 15, 17], [18, 27, 22], [23, 24, 25]]) 
        self.freq = 250


class ServoParams:
    def __init__(self):
        self.min_pwm = 680    # Minimum pulse width in microseconds
        self.max_pwm = 2320   # Maximum pulse width in microseconds
        
        self.micros_per_rad = MICROS_PER_RAD  # Must be calibrated

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle = NEUTRAL_ANGLE_DEGREES * np.pi / 180.0  # Convert to radians

        self.servo_multipliers = SERVO_MULTIPLIERS
        
        self.angle_constraints = {k: np.array(v) * np.pi / 180.0 for k, v in ANGLE_CONSTRAINTS_DEGREES.items()}  # Convert to radians