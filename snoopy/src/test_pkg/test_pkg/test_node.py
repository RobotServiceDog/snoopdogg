import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from comm_utils.msg import LegPosition
from geometry_msgs.msg import Point

import sys
import termios
import tty
import threading

class PointVisualizer(Node):
    def __init__(self):
        super().__init__('point_visualizer_node')
        
        # 1. Publisher for RViz markers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.pub = self.create_publisher(LegPosition, '/leg_position_cmd', 10)
        
        # 2. Hardcoded list of points (X, Y, Z) to cycle through
        self.points_list = [
            (0.032, 0.032, -0.189),
            (0.032, 0.032, -0.139),
            (0.082, 0.032, -0.139),
            (0.082, 0.032, -0.189),
            (0.032, 0.032, -0.189),
        ]
        self.current_index = 0

        self.get_logger().info(
            f"Node started! Press 'k' in this terminal to publish the next point "
            f"(Total points: {len(self.points_list)})"
        )

        # 3. Start a background thread to capture keyboard presses without blocking ROS 2
        self.running = True
        self.input_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.input_thread.start()

    def publish_next_point(self):
        """Publishes the current point as a Sphere Marker in RViz."""
        if self.current_index >= len(self.points_list):
            self.get_logger().warn("Reached the end of the point list! Resetting to the beginning.")
            self.current_index = 0

        x, y, z = self.points_list[self.current_index]

        # Construct the marker message
        marker = Marker()
        marker.header.frame_id = "LF_HIP"  # Ensure this frame exists or matches your RViz fixed frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "points_group"
        marker.id = self.current_index # Unique ID per point so they all stay drawn
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        
        # Orientation (Quaternions - must be normalized!)
        marker.pose.orientation.w = 1.0

        # Scale (Diameter of the sphere)
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # Color (RGBA: values are between 0.0 and 1.0)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # DON'T FORGET ALPHA (Transparency), or it will be invisible!

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published Point [{self.current_index}]: x={x}, y={y}, z={z}")

        msg = LegPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "LF_HIP"
        msg.env = "sim"

        # 2. Calculate trajectories (passing side info for turn logic)
        lf_x, lf_y, lf_z = (float(x), float(y), float(z))
        rf_x, rf_y, rf_z = (0.032, -0.032, -0.189)
        lh_x, lh_y, lh_z = (0.032, 0.032, -0.189)
        rh_x, rh_y, rh_z = (0.032, -0.032, -0.189)

        raw_positions = [lf_x, lf_y, lf_z, rf_x, rf_y, rf_z, lh_x, lh_y, lh_z, rh_x, rh_y, rh_z]

        msg.foot_position = raw_positions
        self.pub.publish(msg)

        self.current_index += 1

    def keyboard_listener(self):
        """Captures a single keypress without requiring enter (Linux/macOS)."""
        while self.running and rclpy.ok():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

            if ch == 'k' or ch == 'K':
                # Trigger the publish
                self.publish_next_point()
            elif ch == '\x03':  # Ctrl+C
                self.get_logger().info("Ctrl+C detected in keyboard listener.")
                self.running = False
                break

def main(args=None):
    rclpy.init(args=args)
    node = PointVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()