"""
Nav2 localization + path planning launch.
Run this AFTER building a map with bringup.launch.py.

Usage:
  ros2 launch agv_navigation nav2_localization.launch.py
  ros2 launch agv_navigation nav2_localization.launch.py map:=/home/bart/maps/agv_map.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

CONFIG_DIR = os.path.join(
    get_package_share_directory('agv_navigation'), 'config'
)
NAV2_PARAMS = os.path.join(CONFIG_DIR, 'nav2_params.yaml')


def generate_launch_description():
    map_yaml = LaunchConfiguration(
        'map', default='/home/bart/maps/agv_map.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/bart/maps/agv_map.yaml',
            description='Full path to the map yaml file',
        ),

        # Serve the saved occupancy map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[NAV2_PARAMS, {'yaml_filename': map_yaml}],
        ),

        # Particle filter localization on the loaded map
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[NAV2_PARAMS],
            remappings=[('scan', '/scan')],
        ),

        # Global path planner (A* — no cmd_vel output)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[NAV2_PARAMS],
        ),

        # Global costmap (needed by planner)
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[NAV2_PARAMS],
            remappings=[('scan', '/scan')],
        ),

        # Lifecycle manager for map_server + amcl + planner
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl', 'planner_server', 'global_costmap'],
            }],
        ),

        # rosbridge WebSocket — lets browser HMI subscribe to ROS topics
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}],
        ),
    ])
