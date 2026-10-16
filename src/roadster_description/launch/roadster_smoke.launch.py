"""roadster_smoke.launch.py - flat-runway smoke test for roadster_description.

Gazebo (roadster_flat_test world) + robot_state_publisher + roadster spawn +
bridge + ros2_control spawners. No planner/MPC/perception: drive it by
publishing to the controllers directly, e.g.

  ros2 topic pub -r 50 /wheel_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [300, 300, 300, 300]}"
  ros2 topic pub --once /steering_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.2, 0.2]}"

  ros2 launch roadster_description roadster_smoke.launch.py
  ros2 launch roadster_description roadster_smoke.launch.py gui:=true
  ros2 launch roadster_description roadster_smoke.launch.py chassis_mass:=300
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    cfg = context.launch_configurations
    pkg = get_package_share_directory('roadster_description')
    world = 'roadster_flat_test'
    world_file = os.path.join(pkg, 'worlds', f'{world}.world.sdf')
    urdf_file = os.path.join(pkg, 'urdf', 'roadster.urdf.xacro')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge_roadster.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file,
                 ' chassis_mass:=', cfg['chassis_mass'],
                 ' tire_mu:=', cfg['tire_mu'],
                 ' with_camera:=', cfg['with_camera']]),
        value_type=str)

    # server-only unless gui:=true; NEVER --headless-rendering (WSL freeze)
    gz_cmd = ['ign', 'gazebo', '-r', world_file]
    if cfg['gui'].lower() != 'true':
        gz_cmd.insert(2, '-s')

    spawner = lambda name: Node(  # noqa: E731
        package='controller_manager', executable='spawner',
        arguments=[name, '--controller-manager', '/controller_manager'],
        output='screen')

    # gz_ros2_control's system plugin lives in /opt/ros/humble/lib, which is
    # not on Gazebo's default system-plugin search path
    plugin_path = os.pathsep.join(filter(None, [
        os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'humble'), 'lib'),
        os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')]))

    # Fortress's contact system ignores the sensor's <topic> and publishes on
    # the scoped default name, which embeds the world - bridged as a separate
    # positional-arg node (a positional arg on the config_file node silently
    # drops the config_file bridges, see urban_world.launch.py)
    contact_gz = (f'/world/{world}/model/roadster/link/base_link/sensor/'
                  'body_contact/contact')

    return [
        ExecuteProcess(cmd=gz_cmd,
                       additional_env={'LIBGL_ALWAYS_SOFTWARE': '1',
                                       'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': plugin_path},
                       output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}]),

        TimerAction(period=3.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='gz_ros_bridge',
                 parameters=[{'config_file': bridge_cfg}],
                 output='screen'),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='gz_ros_bridge_contact',
                 arguments=[f'{contact_gz}@ros_gz_interfaces/msg/Contacts'
                            '[ignition.msgs.Contacts'],
                 remappings=[(contact_gz, '/roadster/body_contact')],
                 output='screen')]),

        # base_link sits at ground level (same as the sedan); spawn 5 cm up
        TimerAction(period=5.0, actions=[
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'roadster', '-world', world,
                            '-topic', 'robot_description',
                            '-x', '0', '-y', '0', '-z', '0.05'],
                 output='screen')]),

        # the controller_manager lives inside Gazebo (gz_ros2_control), so the
        # spawners must wait for the model
        TimerAction(period=9.0, actions=[
            spawner('joint_state_broadcaster'),
            spawner('wheel_effort_controller'),
            spawner('steering_position_controller')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('chassis_mass', default_value='1730'),
        DeclareLaunchArgument('tire_mu', default_value='1.2'),
        DeclareLaunchArgument('with_camera', default_value='false'),
        OpaqueFunction(function=setup),
    ])
