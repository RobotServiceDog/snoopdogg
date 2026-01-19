import rclpy
from rclpy.node import Node
from comm_utils.msg import LegPosition
import numpy as np
import math

class TrotGaitNode(Node):
    def __init__(self):
        super().__init__('trot_gait_provider')
        
        # --- Parameters (Tune these!) ---
        self.declare_parameter('step_frequency', 1.0)    # Hz
        self.declare_parameter('step_length', 0.02)       # Meters (X swing)
        self.declare_parameter('step_height', 0.01)       # Meters (Z lift)
        self.declare_parameter('base_height', 0.15)       # Meters (Z offset from hip)
        self.declare_parameter('env', 'sim')             # 'sim' or 'real'
        
        # New Tuning Parameters for Stability
        self.declare_parameter('warmup_time', 3.0)        # Seconds to transition from home to trot
        self.declare_parameter('home_z', 0.15)            # The Z position where the dog is standing "stiff"

        self.pub = self.create_publisher(LegPosition, '/leg_position_cmd', 10)
        
        self.timer_period = 1.0 / 100.0 # 100 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.start_time = None

    def get_leg_trajectory(self, phase_offset):
        """Calculates X, Y, Z for a single leg based on global phase."""
        freq = self.get_parameter('step_frequency').value
        length = self.get_parameter('step_length').value
        height = self.get_parameter('step_height').value
        base_z = self.get_parameter('base_height').value
        
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        # Calculate the current position in the gait cycle
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        phase = (2 * math.pi * freq * t + phase_offset) % (2 * math.pi)

        if phase <= math.pi:
            # length / 2 to set diameter of unit circle to be 1.0 instead of 2.0
            x = (length / 2) * math.cos(phase)
            z = base_z 
        else:
            x = (length / 2) * math.cos(phase)
            z = base_z - height * math.sin(phase - math.pi) # offset to make sin positive

        return x, 0.0425, z 

    def timer_callback(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            
        msg = LegPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.env = self.get_parameter('env').value

        # Calculate raw trot positions
        lf_x, lf_y, lf_z = self.get_leg_trajectory(0)
        rf_x, rf_y, rf_z = self.get_leg_trajectory(math.pi)
        lh_x, lh_y, lh_z = self.get_leg_trajectory(math.pi)
        rh_x, rh_y, rh_z = self.get_leg_trajectory(0)

        raw_positions = [
            lf_x, -lf_y,  lf_z,   
            rf_x,  rf_y,  rf_z,   
            lh_x, -lh_y,  lh_z,   
            rh_x,  rh_y,  rh_z    
        ]

        # --- SOFT START LOGIC ---
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        warmup = self.get_parameter('warmup_time').value
        home_z = self.get_parameter('home_z').value
        
        # lerp_factor goes from 0.0 to 1.0
        lerp_factor = min(t / warmup, 1.0)

        # Blend from a neutral standing pose (0, Y, home_z) to the trot target
        final_positions = []
        for i in range(12):
            # For X (indices 0, 3, 6, 9) and Y (indices 1, 4, 7, 10), start at home values
            if i % 3 == 2: # Z components
                target_z = raw_positions[i]
                final_positions.append((1 - lerp_factor) * home_z + lerp_factor * target_z)
            elif i % 3 == 0: # X components
                target_x = raw_positions[i]
                final_positions.append(lerp_factor * target_x) # Start X at 0.0
            else: # Y components
                final_positions.append(raw_positions[i]) # Keep Y constant

        msg.foot_position = final_positions
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrotGaitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()