import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
import time

class ErrorPlotter(Node):
    def __init__(self):
        super().__init__('error_plotter')
        self.subscription = self.create_subscription(
            Vector3,
            '/controller_error',
            self.listener_callback,
            10)
        
        # Data storage
        self.times = []
        self.dist_errors = []
        self.angle_errors = []
        self.start_time = time.time()
        
        self.get_logger().info('Plotter node started. Collecting data... Press Ctrl+C to save and exit.')

    def listener_callback(self, msg):
        # Record relative time and error values
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        self.dist_errors.append(msg.x)
        self.angle_errors.append(msg.y)

    def save_plot(self):
        if not self.times:
            print("No data collected!")
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # Distance Error Plot
        ax1.plot(self.times, self.dist_errors, 'r-', label='Distance Error (m)')
        ax1.set_ylabel('Distance Error [m]')
        ax1.set_title('Controller Response Over Time')
        ax1.grid(True)
        ax1.legend()

        # Angle Error Plot
        ax2.plot(self.times, self.angle_errors, 'b-', label='Angle Error (rad)')
        ax2.set_ylabel('Angle Error [rad]')
        ax2.set_xlabel('Time [s]')
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()
        filename = "controller_response.png"
        plt.savefig(filename)
        print(f"Plot saved as {filename}")

def main(args=None):
    rclpy.init(args=args)
    plotter = ErrorPlotter()

    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        # Save the image when the user hits Ctrl+C
        plotter.get_logger().info('Saving plot...')
        plotter.save_plot()
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()