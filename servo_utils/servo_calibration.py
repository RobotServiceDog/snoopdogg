
import numpy as np


MICROS_PER_RAD = 11.3333 * 180.0 / np.pi
NEUTRAL_ANGLE_DEGREES = np.array(
[[  0.,  0.,  0.],
 [  0.,  0.,  0.],
 [  0.,  0.,  0.],
 [  0.,  0.,  0.]])

# [min_pwm, mid_pwm, max_pwm]
SERVO_PWM_RANGES = np.array([
    [
        [680, 1340, 2320], # LF Axis 0 # TODO: Fill in actual calibrated values
        [680, 1340, 2320], # LF Axis 1
        [800, 1550, 2320], # LF Axis 2
    ],
    [
        [680, 1340, 2320], # RF Axis 0 # TODO: Fill in actual calibrated values
        [680, 1340, 2320], # RF Axis 1 # TODO: Fill in actual calibrated values
        [680, 1340, 2320], # RF Axis 2 # TODO: Fill in actual calibrated values
    ],
    [
        [680, 1340, 2320], # LH Axis 0 # TODO: Fill in actual calibrated values
        [680, 1340, 2320], # LH Axis 1 # TODO: Fill in actual calibrated values
        [680, 1340, 2320], # LH Axis 2 # TODO: Fill in actual calibrated values
    ],
    [
        [680, 1340, 2320], # RH Axis 0 # TODO: Fill in actual calibrated values
        [680, 1340, 2320], # RH Axis 1 # TODO: Fill in actual calibrated values
        [680, 1340, 2320], # RH Axis 2 # TODO: Fill in actual calibrated values
    ],
])
    