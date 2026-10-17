"""demo_c.launch.py - Demo C, the straight launch: stunt_demo.launch.py on roadster_launch.

  ros2 launch roadster_control demo_c.launch.py
  ros2 launch roadster_control demo_c.launch.py gui:=false          # RViz only
  ros2 launch roadster_control demo_c.launch.py state:=truth        # fly on Gazebo truth

Stand on the line, then launch on the tyres at their traction limit plus
the rear nozzles at 58.5% throttle: 0-60 mph in about 0.81 s, then brake
to a stop.

Every stunt_demo.launch.py argument (gui, rviz, state, onboard_camera,
unpause_after, start_delay) passes through. Generate the world first if it
is missing:
  python3 scripts/generate_stunt_world.py --kind launch
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
        launch_arguments={'world': 'roadster_launch', 'mode': 'launch'}.items())])
