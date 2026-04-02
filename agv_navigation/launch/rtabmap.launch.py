from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os

CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='true',
            description='Delete RTAB-Map database on start (false to resume mapping)',
        ),

        # RTAB-Map SLAM — map building + geometry loop closure
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[os.path.join(CONFIG_DIR, 'rtabmap.yaml'), {'delete_db_on_start': True}],
            remappings=[
                ('scan_cloud', '/ifm3d/camera/cloud'),
                ('odom',       '/odom'),
            ],
        ),

        # Point cloud -> 2D laser scan for Nav2 costmaps
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[os.path.join(CONFIG_DIR, 'pointcloud_to_laserscan.yaml')],
            remappings=[
                ('cloud_in', '/ifm3d/camera/cloud'),
                ('scan',     '/scan'),
            ],
        ),
    ])
