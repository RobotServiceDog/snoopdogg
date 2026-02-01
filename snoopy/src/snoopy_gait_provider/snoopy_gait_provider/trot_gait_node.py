from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from comm_utils.msg import LegPosition
from geometry_msgs.msg import Twist
import numpy as np
import math

CONTROL_LOOP_FREQ = 100.0

class TrotGaitNode(Node):
    def __init__(self):
        super().__init__('trot_gait_provider')
        
        # --- Parameters ---
        self.step_frequency = self.declare_parameter('step_frequency', 4.0).value    
        self.max_step_length = self.declare_parameter('max_step_length', 0.03).value 
        self.step_height = self.declare_parameter('step_height', 0.02).value       
        self.base_height = self.declare_parameter('base_height', 0.15).value       
        self.env = self.declare_parameter('env', 'sim').value             
        self.warmup_time = self.declare_parameter('warmup_time', 2.0).value        
        self.home_z = self.declare_parameter('home_z', 0.15).value       
        self.max_angular_velocity = self.declare_parameter('max_angular_velocity', 0.1).value     

        # --- Slew Rate / Smoothing Variables ---
        self.current_stride_x = 0.0
        self.target_stride_x = 0.0
        self.current_yaw_rate = 0.0
        self.target_yaw_rate = 0.0
        self.slew_rate = 0.2  # Max change in meters/rad per second
        
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(LegPosition, '/leg_position_cmd', 10)
        
        self.timer_period = 1.0 / CONTROL_LOOP_FREQ 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.start_time = None

    def get_leg_trajectory(self, phase_offset, is_left_side):
        """Calculates X, Y, Z for a single leg based on global phase."""
        freq = self.step_frequency
        height = self.step_height
        base_z = self.base_height
        
        # Dynamic stride and turn math
        # Positive yaw_rate = turn left = left side slower, right side faster
        yaw_offset = self.current_yaw_rate * 0.05  # tuning constant for turn radius
        side_sign = -1.0 if is_left_side else 1.0
        effective_length = self.current_stride_x + (side_sign * yaw_offset)
        
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        phase = (2 * math.pi * freq * t + phase_offset) % (2 * math.pi)

        x = (effective_length / 2) * math.cos(phase)
        if phase <= math.pi:
            # Stance phase
            z = base_z 
        else:
            # Swing phase
            z = base_z - height * math.sin(phase - math.pi)

        return x, 0.0425, z 

    def apply_slew_rate(self):
        """Smooths out the target velocity commands to prevent robot flipping."""
        step = self.slew_rate * self.timer_period
        
        # Slew for Linear X
        diff_x = self.target_stride_x - self.current_stride_x
        if abs(diff_x) < step:
            self.current_stride_x = self.target_stride_x
        else:
            self.current_stride_x += math.copysign(step, diff_x)
            
        # Slew for Angular Z
        diff_z = self.target_yaw_rate - self.current_yaw_rate
        if abs(diff_z) < step:
            self.current_yaw_rate = self.target_yaw_rate
        else:
            self.current_yaw_rate += math.copysign(step, diff_z)

    def timer_callback(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        # 1. Smooth the velocity inputs
        self.apply_slew_rate()
            
        msg = LegPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.env = self.env

        # 2. Calculate trajectories (passing side info for turn logic)
        lf_x, lf_y, lf_z = self.get_leg_trajectory(0, True)        # LF (Left)
        rf_x, rf_y, rf_z = self.get_leg_trajectory(math.pi, False) # RF (Right)
        lh_x, lh_y, lh_z = self.get_leg_trajectory(math.pi, True)  # LH (Left)
        rh_x, rh_y, rh_z = self.get_leg_trajectory(0, False)       # RH (Right)

        raw_positions = [lf_x, -lf_y, lf_z, rf_x, rf_y, rf_z, lh_x, -lh_y, lh_z, rh_x, rh_y, rh_z]

        # 3. Apply Soft Start
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        lerp_factor = min(t / self.warmup_time, 1.0)

        final_positions = []
        for i in range(12):
            if i % 3 == 2: # Z
                final_positions.append((1 - lerp_factor) * self.home_z + lerp_factor * raw_positions[i])
            elif i % 3 == 0: # X
                final_positions.append(lerp_factor * raw_positions[i])
            else: # Y
                final_positions.append(raw_positions[i])

        msg.foot_position = final_positions
        self.pub.publish(msg)
    
    def cmd_vel_callback(self, msg):
        # We don't update current_stride directly; we set the target for the slew filter
        self.target_stride_x = min(max(msg.linear.x * 0.1, -self.max_step_length), self.max_step_length)
        self.target_yaw_rate = min(max(msg.angular.z, -self.max_angular_velocity), self.max_angular_velocity)

def main(args=None):
    rclpy.init(args=args)
    node = TrotGaitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()