import numpy as np
import math
import matplotlib.pyplot as plt

CONTROL_LOOP_FREQ = 100.0

class TrotGaitSimulator:
    def __init__(self):
        # --- Parameters ---
        self.step_frequency = 2.0    
        self.max_step_length = 0.1
        self.step_height = 0.05       
        self.base_height = 0.17       
        self.warmup_time = 1.0  
        self.home_z = 0.15       
        self.max_angular_velocity = 0.1
        self.dt = 0.01  

        self.current_stride_x = 0.0
        self.target_stride_x = 0.0
        self.current_yaw_rate = 0.0
        self.target_yaw_rate = 0.0
        self.slew_rate = 0.2 

        self.timer_period = 1.0 / CONTROL_LOOP_FREQ
        
    def set_velocity(self, vx, yaw):
        self.target_stride_x = min(max(vx * 0.1, -self.max_step_length), self.max_step_length)
        self.target_yaw_rate = min(max(yaw, -self.max_angular_velocity), self.max_angular_velocity)

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

    def get_leg_trajectory(self, t, phase_offset, is_left_side):
        """Calculates X, Y, Z for a single leg based on global phase."""
        freq = self.step_frequency
        height = self.step_height
        base_z = self.base_height
        
        # Dynamic stride and turn math
        # Positive yaw_rate = turn left = left side slower, right side faster
        yaw_offset = self.current_yaw_rate * 0.05  # tuning constant for turn radius
        side_sign = -1.0 if is_left_side else 1.0
        effective_length = self.current_stride_x + (side_sign * yaw_offset)
        
        phase = (2 * math.pi * freq * t + phase_offset) % (2 * math.pi)

        x = (effective_length / 2) * math.cos(phase)
        if phase <= math.pi:
            # Stance phase
            z = base_z 
        else:
            # Swing phase
            z = base_z - height * math.sin(phase - math.pi)

        return x, z 

    def run_simulation(self, duration=2.5):
        steps = int(duration / self.dt)
        time_axis = np.linspace(0, duration, steps)
        results = { "LF": [], "RF": [], "LH": [], "RH": [] }

        for t in time_axis:
            self.apply_slew_rate()
            legs = {
                "LF": self.get_leg_trajectory(t, 0, True),
                "RF": self.get_leg_trajectory(t, math.pi, False),
                "LH": self.get_leg_trajectory(t, math.pi, True),
                "RH": self.get_leg_trajectory(t, 0, False)
            }
            lerp_factor = min(t / self.warmup_time, 1.0)
            for leg_id, (raw_x, raw_z) in legs.items():
                final_x = lerp_factor * raw_x
                final_z = (1 - lerp_factor) * self.home_z + lerp_factor * raw_z
                results[leg_id].append((raw_x, raw_z))
        
        return time_axis, results

    def plot_time_series(self, time_axis, results):
        """Figure 1: Position vs Time."""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        fig.canvas.manager.set_window_title('Gait Analysis: Time Series')

        for leg, data in results.items():
            data = np.array(data)
            ax1.plot(time_axis, data[:, 0], label=f'{leg} X')
            ax2.plot(time_axis, data[:, 1], label=f'{leg} Z')

        ax1.set_title(f"Leg Position Over Time [Forward Velocity: {self.target_stride_x:.3f} m, Angular Velocity: {self.target_yaw_rate:.3f} rad/s]")
        ax1.set_ylabel("X (Forward) [m]")
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right', ncol=2)

        ax2.set_ylabel("Z (Height) [m]")
        ax2.set_xlabel("Time [s]")
        ax2.invert_yaxis() 
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right', ncol=2)
        
    def plot_side_view(self, results):
        """Figure 2: Spatial Trajectory (X vs Z) with parabola pointing down."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10), sharex=True)
        fig.canvas.manager.set_window_title('Gait Analysis: Parabola Down')
        
        ax_list = axes.flatten()
        colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange']

        for i, (leg, data) in enumerate(results.items()):
            ax = ax_list[i]
            ax.invert_yaxis()
            data = np.array(data)
            
            # Plot X vs Z
            ax.plot(data[:, 0], data[:, 1], color=colors[i], lw=3, label=leg)
            ax.scatter(data[-1, 0], data[-1, 1], color='black', zorder=5)

            ax.set_title(f"Leg: {leg}")
            ax.grid(True, linestyle='--', alpha=0.5)
            
            # Aspect 'equal' with 'box' adjustment prevents the shared axis crash
            # --- THE FIX ---
            # DO NOT use ax.invert_yaxis(). 
            # By default, Matplotlib puts 0.15 (Ground) ABOVE 0.13 (Swing Peak).
            # This will result in the 'downward' curving parabola you want.

        # set shared x and z label for the plot
        for ax in ax_list:
            ax.set_xlabel("X (Forward) [m]")
            ax.set_ylabel("Z (Height) [m]")
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    sim = TrotGaitSimulator()
    sim.set_velocity(0.5, 0.0) # Set forward velocity
    
    t_axis, data_results = sim.run_simulation(duration=2.0)
    
    sim.plot_time_series(t_axis, data_results)
    sim.plot_side_view(data_results)
    
    # Crucial: plt.show() blocks execution, so call it once at the end
    plt.show()