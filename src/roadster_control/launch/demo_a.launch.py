"""demo_a.launch.py - Demo A, the inverted ramp: stunt_demo.launch.py on roadster_ramp.

  ros2 launch roadster_control demo_a.launch.py
  ros2 launch roadster_control demo_a.launch.py gui:=false          # RViz only
  ros2 launch roadster_control demo_a.launch.py state:=truth        # fly on Gazebo truth
  ros2 launch roadster_control demo_a.launch.py release_after_hold:=false   # climb and stay held

Run-in, up the 8 m loop on fan suction, stop upside down on the overhang,
hold, drive off it still inverted, kick-flip 180 deg on the cold-gas
nozzles, landing burn and flare, land on the wheels.

Every stunt_demo.launch.py argument (gui, rviz, state, onboard_camera,
unpause_after, start_delay) passes through. Generate the world first if it
is missing:
  python3 scripts/generate_stunt_world.py --kind ramp --radius 8
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
        launch_arguments={'world': 'roadster_ramp', 'mode': 'ramp'}.items())])
