"""stunt_mission.launch.py - the roadster stunt mission node (needs a running sim).

  ros2 launch roadster_control stunt_mission.launch.py               # mode from stunt_params.yaml
  ros2 launch roadster_control stunt_mission.launch.py mode:=drop
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('roadster_control'),
                          'config', 'stunt_params.yaml')
    nozzles = os.path.join(get_package_share_directory('roadster_description'),
                           'config', 'nozzles.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='drop'),
        Node(package='roadster_control', executable='stunt_mission_node',
             name='stunt_mission_node', output='screen',
             parameters=[params, {'use_sim_time': True,
                                  'nozzles_file': nozzles,
                                  'mode': LaunchConfiguration('mode')}]),
    ])
