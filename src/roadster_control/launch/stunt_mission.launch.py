"""stunt_mission.launch.py - the roadster stunt mission node (needs a running sim).

  ros2 launch roadster_control stunt_mission.launch.py mode:=drop
  ros2 launch roadster_control stunt_mission.launch.py mode:=ramp world:=roadster_ramp
  ros2 launch roadster_control stunt_mission.launch.py mode:=ramp world:=roadster_ramp \\
      release_after_hold:=false        # Phase 4: climb and stay held upside down

The world's geometry comes from ~/vlm_av_ws/maps/<world>.json (written by
scripts/generate_stunt_world.py, the single owner).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    cfg = context.launch_configurations
    params = os.path.join(get_package_share_directory('roadster_control'),
                          'config', 'stunt_params.yaml')
    nozzles = os.path.join(get_package_share_directory('roadster_description'),
                           'config', 'nozzles.yaml')
    overrides = {'use_sim_time': True, 'nozzles_file': nozzles, 'mode': cfg['mode'],
                 'world_json': os.path.expanduser(f'~/vlm_av_ws/maps/{cfg["world"]}.json')}
    if cfg['release_after_hold'] != '':
        overrides['release_after_hold'] = cfg['release_after_hold'].lower() == 'true'
    if cfg['start_delay'] != '':
        overrides['start_delay'] = float(cfg['start_delay'])
    use_est = cfg['state'] == 'imu'
    overrides['odom_topic'] = '/odom_est' if use_est else '/odom_truth'
    # the IMU estimator always runs (so its error against truth can be scored);
    # state:=imu makes the mission fly on it instead of on ground truth
    estimator = Node(package='roadster_control', executable='state_estimator_node',
                     name='state_estimator', output='screen',
                     parameters=[{'use_sim_time': True,
                                  'world_json': overrides['world_json']}])
    return [estimator,
            Node(package='roadster_control', executable='stunt_mission_node',
                 name='stunt_mission_node', output='screen',
                 parameters=[params, overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='drop'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('release_after_hold', default_value='',
                              description="'' = stunt_params.yaml value"),
        DeclareLaunchArgument('state', default_value='truth',
                              description="'truth' = /odom_truth, 'imu' = /odom_est "
                                          '(IMU strapdown EKF)'),
        DeclareLaunchArgument('start_delay', default_value='',
                              description="sim seconds before the mission starts; '' = yaml"),
        OpaqueFunction(function=setup),
    ])
