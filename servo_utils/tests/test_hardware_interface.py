# sweep_demo_fixed.py
# A runnable demo script that sweeps a single actuator (0, 0) across a range of angles.

from servo_utils.hardware_interface import HardwareInterface
from servo_utils.config import NUM_LEGS

import time
import math

def test_sweep_actuator():
    """Demo function to sweep a single actuator (0, 0) through a range of angles."""
    
    hw_interface = None # Initialize to None for cleanup safety
    try:
        hw_interface = HardwareInterface()
        print("Hardware Interface initialized. Starting single actuator sweep...")
        
        # Sweep parameters
        min_angle_deg = -45
        max_angle_deg = 45
        step_deg = 1
        
        axis, leg = 0, 0  # Test first actuator (0, 0)
        
        num_iters = 10
        print(f"Sweeping actuator ({axis}, {leg}) from {min_angle_deg}° to {max_angle_deg}° for {num_iters} iterations.")
        
        for i in range(num_iters):
            
            # Sweep Forward (min to max)
            for angle_deg in range(min_angle_deg, max_angle_deg + 1, step_deg):
                angle_rad = math.radians(angle_deg)
                hw_interface.set_actuator_position(angle_rad, axis, leg)
                
                # Logging the current command
                print(f"\r[I{i+1}/{num_iters}] Commanding ({axis}, {leg}) to: {angle_deg}°", end="", flush=True)
                time.sleep(0.01)
                
            # Sweep Backward (max to min)
            for angle_deg in range(max_angle_deg, min_angle_deg - 1, -step_deg):
                angle_rad = math.radians(angle_deg)
                hw_interface.set_actuator_position(angle_rad, axis, leg)
                
                # Logging the current command
                print(f"\r[I{i+1}/{num_iters}] Commanding ({axis}, {leg}) to: {angle_deg}°", end="", flush=True)
                time.sleep(0.01)
                
            
        print("\nSweep cycles complete.")
            
    except Exception as e:
        print(f"\n\nAn error occurred during sweep: {e}")
        
    finally:
        if hw_interface:
            hw_interface.close()
            print("Interface closed and actuator signal disabled.")
    
if __name__ == "__main__":
    test_sweep_actuator()
