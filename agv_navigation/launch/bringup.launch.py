"""
Full system bringup: camera + IMU + description + RTAB-Map.
Excludes Nav2 (run separately after map is built).

Usage:
  ros2 launch agv_navigation bringup.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    agv_nav_share = get_package_share_directory('agv_navigation')
    agv_desc_share = get_package_share_directory('agv_description')
    ifm3d_share = get_package_share_directory('ifm3d_ros2')

    return LaunchDescription([
        # 1. ifm3d camera node (port0 3D + port2 RGB)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ifm3d_share, 'launch', 'camera.launch.py')
            ),
        ),

        # 2. Robot state publisher (TF tree)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(agv_desc_share, 'launch', 'description.launch.py')
            ),
        ),

        # 3. IMU publisher — port6 PCIC, publishes /imu/data at ~1kHz
        Node(
            package='agv_imu',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[{
                'ip': '10.167.53.27',
                'pcic_port': 50016,
                'frame_id': 'imu_link',
            }],
        ),

        # 4. ICP odometry — point cloud odometry, publishes to /odom_icp (EKF fuses this)
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[os.path.join(agv_nav_share, 'config', 'icp_odometry.yaml')],
            remappings=[
                ('scan_cloud', '/ifm3d/camera/cloud'),
                ('odom',       '/odom_icp'),
            ],
        ),

        # 5. robot_localization EKF — fuses ICP odom + IMU -> /odom -> base_footprint TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(agv_nav_share, 'config', 'ekf_params.yaml')],
            remappings=[('odometry/filtered', '/odom')],
        ),

        # Foxglove: run separately with:
        #   ros2 launch foxglove_bridge foxglove_bridge_launch.xml

        # 6. RTAB-Map visualiser GUI
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[os.path.join(agv_nav_share, 'config', 'rtabmap.yaml')],
            remappings=[
                ('scan_cloud', '/ifm3d/camera/cloud'),
                ('odom',       '/odom'),
            ],
        ),

        # 7. RTAB-Map (SLAM + pointcloud_to_laserscan, odometry from EKF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(agv_nav_share, 'launch', 'rtabmap.launch.py')
            ),
        ),
    ])
