
import numpy as np

from servo_utils.servo_calibration import MICROS_PER_RAD, NEUTRAL_ANGLE_DEGREES

NUM_AXES = 3
NUM_LEGS = 4

class PWMParams:
    def __init__(self):        
        self.pins = np.array([[2, 14, 18, 23], [3, 15, 27, 24], [4, 17, 22, 25]])
        self.freq = 250


class ServoParams:
    def __init__(self):
        self.min_pwm = 680    # Minimum pulse width in microseconds
        self.mid_pwm = 1500  # Middle position
        self.max_pwm = 2320   # Maximum pulse width in microseconds
        
        self.micros_per_rad = MICROS_PER_RAD  # Must be calibrated

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = NEUTRAL_ANGLE_DEGREES

        self.servo_multipliers = np.array(
            [[1, 1, 1, 1], [-1, 1, -1, 1], [1, -1, 1, -1]]
        )

    @property
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians
