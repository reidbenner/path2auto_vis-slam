"""Launch RGB camera node + person detection with proximity warning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_person_detection')
    ifm3d_pkg_dir = get_package_share_directory('ifm3d_ros2')

    detection_config = os.path.join(pkg_dir, 'config', 'person_detection.yaml')
    rgb_camera_config = os.path.join(pkg_dir, 'config', 'camera_rgb.yaml')

    # RGB camera lifecycle node (port 2, PCIC 50012)
    rgb_camera_node = LifecycleNode(
        package='ifm3d_ros2',
        executable='camera_standalone',
        namespace='ifm3d',
        name='camera_rgb',
        output='screen',
        parameters=[rgb_camera_config],
    )

    # Lifecycle: configure then activate
    configure_rgb = TimerAction(
        period=1.0,
        actions=[EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(rgb_camera_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )]
    )

    activate_rgb = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=rgb_camera_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(rgb_camera_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # Person detection node
    person_detector_node = Node(
        package='agv_person_detection',
        executable='person_detector',
        name='person_detector',
        parameters=[detection_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=detection_config,
            description='Path to person detection parameters'
        ),
        rgb_camera_node,
        configure_rgb,
        activate_rgb,
        person_detector_node,
    ])
