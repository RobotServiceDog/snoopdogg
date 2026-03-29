
import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import Float64MultiArray # Assuming joint angles come in an array
from numpy import pi
import pigpio

# Local imports - ensure these are in your python path
from servo_utils.config import PWMParams, ServoParams, NUM_AXES, NUM_LEGS
from servo_utils.servo_calibration import SERVO_PWM_RANGES

class ServoLifecycleNode(Node):
    def __init__(self, node_name="servo_controller"):
        super().__init__(node_name)
        self.pi = None
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.subscription = None
        self.get_logger().info("Node initialized in 'Unconfigured' state.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Initialize pigpio and setup PWM frequencies."""
        self.get_logger().info("Configuring: Initializing pigpio and pins...")
        
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpiod!")
            return TransitionCallbackReturn.FAILURE

        # Initialize PWM Frequencies
        for leg in range(NUM_LEGS):
            for axis in range(NUM_AXES):
                self.pi.set_PWM_frequency(
                    self.pwm_params.pins[leg][axis], self.pwm_params.freq
                )
        
        # Create subscription (but don't act on data until active)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angles',
            self.listener_callback,
            10
        )
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start accepting commands and move servos to neutral/home."""
        self.get_logger().info("Activating: Servos powered.")
        # Optional: Move to a safe home position on startup
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop sending PWM but keep configuration."""
        self.get_logger().info("Deactivating: Stopping all servos.")
        self.stop_all_servos()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Shut down pigpio connection."""
        self.get_logger().info("Cleaning up: Closing pigpio.")
        self.stop_all_servos()
        if self.pi:
            self.pi.stop()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Final emergency shutdown."""
        self.stop_all_servos()
        if self.pi:
            self.pi.stop()
        return TransitionCallbackReturn.SUCCESS

    # --- Logic from your original HardwareInterface ---

    def listener_callback(self, msg):
        """Only process commands if the node is ACTIVE."""
        if self.lifecycle_state.id != State.PRIMARY_STATE_ACTIVE:
            return

        try:
            # Reshape Float64MultiArray to [NUM_LEGS][NUM_AXES]
            # Assumes input is a flat array of size NUM_LEGS * NUM_AXES
            joint_angles = [msg.data[i:i + NUM_AXES] for i in range(0, len(msg.data), NUM_AXES)]
            self.set_actuator_positions(joint_angles)
        except Exception as e:
            self.get_logger().warn(f"Command failed: {e}")

    def set_actuator_positions(self, joint_angles):
        # Your original logic
        self.check_angle_constraints(joint_angles)
        for leg in range(NUM_LEGS):
            for axis in range(NUM_AXES):
                self.send_servo_command(joint_angles[leg][axis], leg, axis)

    def send_servo_command(self, angle, leg, axis):
        pwm_value = self.angle_to_pwm(angle, leg, axis)
        
        # Calibration bounds check
        min_bound, _, max_bound = SERVO_PWM_RANGES[leg][axis]
        if not (min_bound <= pwm_value <= max_bound):
            raise ValueError(f"PWM {pwm_value} out of calibrated bounds")

        self.pi.set_servo_pulsewidth(self.pwm_params.pins[leg][axis], pwm_value)

    def angle_to_pwm(self, angle, leg, axis):
        neutral_angle = self.servo_params.neutral_angles[leg][axis]
        _, neutral_pwm, _ = SERVO_PWM_RANGES[leg][axis]
        micros_per_rad = self.servo_params.micros_per_rad
        multiplier = self.servo_params.servo_multipliers[leg][axis]

        return int(neutral_pwm + multiplier * micros_per_rad * (angle - neutral_angle))

    def stop_all_servos(self):
        if self.pi and self.pi.connected:
            for leg in range(NUM_LEGS):
                for axis in range(NUM_AXES):
                    self.pi.set_servo_pulsewidth(self.pwm_params.pins[leg][axis], 0)

    def check_angle_constraints(self, joint_angles):
        # Insert your original constraint logic here
        pass

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = ServoLifecycleNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()