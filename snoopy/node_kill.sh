# Kill any existing lifecycle nodes to ensure a clean start
pkill -f component_container # Kill any existing component_container processes
pkill -f inverse_kinematics_lifecycle_node # Kill any existing inverse_kin
pkill -f apriltag_detection_lifecycle_node # Kill any existing apriltag_detection