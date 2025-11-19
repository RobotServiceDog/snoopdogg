# cli_servo_commander.py
# A runnable script that takes 12 joint angles (in degrees) via CLI input
# and commands the hardware interface to set the servo positions.

import time
import math
import numpy as np
from servo_utils.hardware_interface import HardwareInterface
from servo_utils.config import NUM_AXES, NUM_LEGS

def cli_servo_commander():
    """
    Initializes the hardware interface, enters a command loop to read
    12 joint angles from the CLI, and publishes them to the servos.
    """
    
    hw_interface = None # Initialize to None for cleanup safety
    expected_count = NUM_AXES * NUM_LEGS
    
    # 1. Open hardware interface cleanly
    try:
        hw_interface = HardwareInterface()
        print("Hardware Interface initialized and ready.")
        print("-" * 50)
        
        # 2. Main command loop
        while True:
            try:
                # Prompt for input
                print(f"Enter {expected_count} joint angles (in degrees, space-separated):")
                print(f"Order: Axis 0 (Legs 0-3) -> Axis 1 (Legs 0-3) -> Axis 2 (Legs 0-3).")
                
                # Get user input
                user_input = input("Angles: ")
                
                # Check for exit command
                if user_input.lower() in ['exit', 'quit', 'q']:
                    print("Exiting command loop.")
                    break
                    
                # Parse input string into a list of degrees
                angles_deg_flat = [float(val) for val in user_input.split()]
                
                # Validate input count
                if len(angles_deg_flat) != expected_count:
                    print(f"Error: Expected {expected_count} angles, but received {len(angles_deg_flat)}.")
                    continue
                
                # Convert to radians and reshape into the (NUM_AXES, NUM_LEGS) structure (3x4)
                # NumPy is used for efficient conversion and reshaping.
                angles_rad_flat = np.radians(angles_deg_flat)
                joint_angles_rad = angles_rad_flat.reshape((NUM_AXES, NUM_LEGS))
                
                # 3. Publish to the servo
                hw_interface.set_actuator_positions(joint_angles_rad)
                
                print(f"Command published successfully at {time.strftime('%H:%M:%S')}")
                print("-" * 50)

            except ValueError as e:
                # Handle non-numeric input or parsing errors
                print(f"Input Error: Please enter 12 space-separated numbers. Details: {e}")
            except Exception as e:
                # Handle any error during the set_actuator_positions call
                print(f"An error occurred while commanding servos: {e}")
                
    except Exception as e:
        print(f"\n\nFATAL: Failed to initialize Hardware Interface: {e}")
        
    # 4. Handle shutdown cleanly
    finally:
        if hw_interface:
            hw_interface.close()
            print("Interface closed and all actuator signals disabled.")
    
if __name__ == "__main__":
    cli_servo_commander()