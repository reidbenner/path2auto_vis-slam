"""
Full system bringup: camera + description + ORB-SLAM3.
Excludes Nav2 (run separately after map is built).

Usage:
  ros2 launch agv_navigation_orbslam bringup.launch.py
  ros2 launch agv_navigation_orbslam bringup.launch.py enable_pangolin:=true
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    agv_nav_share = get_package_share_directory('agv_navigation_orbslam')
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

        # 3. IMU publisher — disabled, port6 PCIC not accessible via FrameGrabber
        # Node(package='agv_imu', executable='imu_publisher', ...),

        # Foxglove: run separately with:
        #   ros2 launch foxglove_bridge foxglove_bridge_launch.xml

        # 4. ORB-SLAM3 (RGBD SLAM + pointcloud_to_laserscan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(agv_nav_share, 'launch', 'orbslam3.launch.py')
            ),
        ),
    ])
