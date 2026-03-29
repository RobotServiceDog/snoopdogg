from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
from lifecycle_msgs.msg import State
import event_handlers

def generate_launch_description():
    # 1. Define the Node
    servo_node = LifecycleNode(
        package='servo_driver',
        executable='servo_node',
        name='servo_controller', # Must match the node name in your Python code
        namespace='',
        output='screen',
        emulate_tty=True
    )

    # 2. Trigger: When node reaches 'Unconfigured', move to 'Inactive' (Configure)
    to_inactive = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.process.matches_action(servo_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # 3. Trigger: When node reaches 'Inactive', move to 'Active' (Activate)
    to_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=servo_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.process.matches_action(servo_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        servo_node,
        to_inactive,
        to_active
    ])