"""demo_b.launch.py - Demo B, the obstacle hop: stunt_demo.launch.py on roadster_hop.

  ros2 launch roadster_control demo_b.launch.py
  ros2 launch roadster_control demo_b.launch.py gui:=false          # RViz only
  ros2 launch roadster_control demo_b.launch.py state:=truth        # fly on Gazebo truth

Drive at 15 m/s toward a row of parked vehicles (Model 3, Model Y, Semi,
Cybertruck, Cybercab), lift off on the bottom nozzles at the planned burn
start, roll a full 360 deg over the row (1 m over the 4.1 m Semi), landing
burn and flare, land on the wheels past the row.

Every stunt_demo.launch.py argument (gui, rviz, state, onboard_camera,
unpause_after, start_delay) passes through. Generate the world first if it
is missing:
  python3 scripts/generate_stunt_world.py --kind hop
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    demo = os.path.join(get_package_share_directory('roadster_control'), 'launch',
                        'stunt_demo.launch.py')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo),
        launch_arguments={'world': 'roadster_hop', 'mode': 'hop'}.items())])
