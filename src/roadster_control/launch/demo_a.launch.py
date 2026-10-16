"""demo_a.launch.py - Demo A, the inverted ramp: Gazebo + RViz + the stunt mission.

  ros2 launch roadster_control demo_a.launch.py
  ros2 launch roadster_control demo_a.launch.py gui:=false          # RViz only
  ros2 launch roadster_control demo_a.launch.py rviz:=false
  ros2 launch roadster_control demo_a.launch.py release_after_hold:=false   # climb and stay held

Sequence: the world starts PAUSED while Gazebo, the bridges, ros2_control and
RViz come up; after `unpause_after` wall seconds it is unpaused and, after a
further `start_delay` sim seconds, the car drives: run-in -> up the 8 m loop
on fan suction -> stops upside down on the overhang -> holds 2 s -> lets go,
kick-flips 180 deg on its cold-gas nozzles -> brake burn -> lands.

The mission flies on the IMU state estimate (state:=truth for Gazebo truth).
RViz shows the car, the ramp (from maps/<world>.json), one orange arrow per
nozzle (length ~ thrust), the air flow (white exhaust puffs, blue fan exhaust,
a fan disc glowing with fan speed and suction streaks into it), a state panel,
the true (yellow) and estimated (orange) path trails, IMU arrows and readout,
control arrows (requested / allocated thrust, torque, wheel torques) and a
control panel, and two Gazebo-rendered camera panels: the trackside camera
filming the stunt and the car's onboard camera (onboard_camera:=false to save
render time). In Gazebo the nozzles and fan exhaust emit particle plumes.
Generate the world first if it is missing:
  python3 scripts/generate_stunt_world.py --kind ramp --radius 8
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    cfg = context.launch_configurations
    world = cfg['world']
    ctrl = get_package_share_directory('roadster_control')
    desc = get_package_share_directory('roadster_description')
    world_json = os.path.join(desc, 'maps', f'{world}.json')
    if not os.path.isfile(world_json):
        raise RuntimeError(f'{world_json} missing: run scripts/generate_stunt_world.py first')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'roadster_sim.launch.py')),
        launch_arguments={'world': world, 'gui': cfg['gui'], 'paused': 'true',
                          'with_camera': cfg['onboard_camera']}.items())
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl, 'launch', 'stunt_mission.launch.py')),
        launch_arguments={'mode': 'ramp', 'world': world,
                          'release_after_hold': cfg['release_after_hold'],
                          'start_delay': cfg['start_delay'], 'state': cfg['state']}.items())
    viz = Node(package='roadster_control', executable='stunt_viz.py', name='stunt_viz',
               output='screen',
               parameters=[{'use_sim_time': True, 'world_json': world_json,
                            'nozzles_file': os.path.join(desc, 'config', 'nozzles.yaml')}])
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                arguments=['-d', os.path.join(ctrl, 'config', 'demo_a.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))
    unpause = ExecuteProcess(
        cmd=['ign', 'service', '-s', f'/world/{world}/control',
             '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '5000', '--req', 'pause: false'],
        output='screen')

    return [
        sim,
        # after the sim's own bring-up (bridges 3 s, spawn 5 s, spawners 9 s)
        TimerAction(period=8.0, actions=[mission, viz, rviz]),
        TimerAction(period=float(cfg['unpause_after']), actions=[unpause]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='roadster_ramp'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('state', default_value='imu',
                              description="'imu' = fly on the IMU state estimate, 'truth' = Gazebo"),
        DeclareLaunchArgument('onboard_camera', default_value='true',
                              description="the car's front camera (shown in RViz); "
                                          'costs real-time factor under software GL'),
        DeclareLaunchArgument('release_after_hold', default_value='true'),
        DeclareLaunchArgument('unpause_after', default_value='14.0',
                              description='wall seconds before the world is unpaused'),
        DeclareLaunchArgument('start_delay', default_value='2.0',
                              description='sim seconds after unpausing before the car drives '
                                          '(ros2_control activates once the world steps)'),
        OpaqueFunction(function=setup),
    ])
