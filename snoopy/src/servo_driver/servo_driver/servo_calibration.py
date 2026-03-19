
import numpy as np


MICROS_PER_RAD = 11.3333 * 180.0 / np.pi
NEUTRAL_ANGLE_DEGREES = np.array(
[[  0.,  0.,  0.],
 [  0.,  0.,  0.],
 [  0.,  0.,  0.],
 [  0.,  0.,  0.]])

SERVO_MULTIPLIERS = np.array(
    [[1, 1, -1], [1, 1, -1], [1, 1, -1], [1, 1, -1]]
)

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


'''
Set the inter-axes constraints on angles (in degrees) to prevent mechanical collisions.

Convention:
    leg:
        0: Left Front (LF)
        1: Right Front (RF)
        2: Left Hind (LH)
        3: Right Hind (RH)
    axis:
        0: Hip (abduction/adduction)
        1: Thigh (hip flexion/extension)
        2: Knee (knee flexion/extension)
    
Example Constraint:
    Theta t between Thigh/Knee on Leg0 must be: 30 <= t <= 150.
    Where the delta = 90 degrees defines the zero-angle offset between the two axes.
    
    Then:
        ANGLE_CONSTRAINTS_DEGREES[(0, 1, 2)] = (30.0, 150.0, 90.0)
'''
ANGLE_CONSTRAINTS_DEGREES = {
    # (leg, axis_from, axis_to): (min_angle_deg, max_angle_deg, axis_delta_deg)
    (0, 1, 2): [30.0, 150.0, 90.0],  # LF Thigh-Knee
}

