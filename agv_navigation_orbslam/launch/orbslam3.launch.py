"""
ORB-SLAM3 RGBD + pointcloud_to_laserscan for Nav2.
Replaces rtabmap.launch.py from agv_navigation.

Publishes:
  /odom              (nav_msgs/Odometry)
  /orb_slam3/path    (nav_msgs/Path)
  /orb_slam3/map_points (sensor_msgs/PointCloud2)
  /scan              (sensor_msgs/LaserScan, from map_points -> laserscan)
  TF: map -> odom -> base_link
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

ORB_SLAM3_ROOT = os.path.join(os.path.expanduser('~'), 'ORB_SLAM3')
CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary_path',
            default_value=os.path.join(ORB_SLAM3_ROOT, 'Vocabulary', 'ORBvoc.txt'),
            description='Path to ORB vocabulary file',
        ),
        DeclareLaunchArgument(
            'settings_path',
            default_value=os.path.join(CONFIG_DIR, 'o3r_orbslam3.yaml'),
            description='Path to ORB-SLAM3 camera settings file',
        ),
        DeclareLaunchArgument(
            'enable_pangolin',
            default_value='false',
            description='Enable Pangolin viewer (set true for debugging)',
        ),

        # ORB-SLAM3 RGBD node — produces /odom + TF + map_points
        Node(
            package='orb_slam3_ros2',
            executable='rgbd_node',
            name='orb_slam3',
            output='screen',
            parameters=[{
                'vocabulary_path': LaunchConfiguration('vocabulary_path'),
                'settings_path': LaunchConfiguration('settings_path'),
                'enable_pangolin': LaunchConfiguration('enable_pangolin'),
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
            }],
            remappings=[
                # Map ifm3d topics to ORB-SLAM3 expected inputs
                ('/camera/rgb/image_raw', '/ifm3d/camera/rgb_2d'),
                ('/camera/depth/image_raw', '/ifm3d/camera/distance'),
            ],
        ),

        # ORB-SLAM3 map_points (3D) -> 2D laser scan for Nav2 costmaps
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[os.path.join(CONFIG_DIR, 'pointcloud_to_laserscan.yaml')],
            remappings=[
                ('cloud_in', '/ifm3d/camera/cloud'),
                ('scan', '/scan'),
            ],
        ),
    ])
