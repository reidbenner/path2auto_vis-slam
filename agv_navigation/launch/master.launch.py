"""
AGV Master Bringup — Terminal 1 of 3.

Launches everything except Nav2 and the HMI web server:
  - ifm3d camera (port0 — 3D ToF cloud)
  - Robot state publisher (TF tree)
  - RTAB-Map: ICP odometry + SLAM + pointcloud_to_laserscan
  - ifm3d camera_rgb (port2 — RGB, lifecycle managed)
  - Person detector (YOLOv8 + proximity + auto-stop)
  - HMI instruction node (turn-by-turn directions)

Terminal 2:
  ros2 launch agv_navigation nav2_localization.launch.py
  (includes rosbridge on :9090)

Terminal 3:
  python3 -m http.server 8080 --directory ~/colcon_ws/src/agv_hmi/hmi
  then open http://localhost:8080
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    agv_nav_share    = get_package_share_directory('agv_navigation')
    agv_person_share = get_package_share_directory('agv_person_detection')

    return LaunchDescription([
        # ---- Camera (3D ToF) + SLAM + description ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(agv_nav_share, 'launch', 'bringup.launch.py')
            ),
        ),

        # ---- RGB camera (port2) + person detector ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(agv_person_share, 'launch', 'person_detection.launch.py')
            ),
        ),

        # ---- HMI instruction node (turn-by-turn directions) ----
        Node(
            package='agv_hmi',
            executable='instruction_node',
            name='instruction_node',
            output='screen',
        ),
    ])
