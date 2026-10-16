"""roadster_sim.launch.py - Gazebo + the stunt roadster, no driving stack.

Gazebo (a roadster world) + robot_state_publisher + roadster spawn + bridges +
ros2_control spawners. Nothing commands the car; tests and the stunt mission
node publish to the controllers / actuation topics directly, e.g.

  ros2 topic pub -r 50 /wheel_effort_controller/commands \\
      std_msgs/msg/Float64MultiArray "{data: [300, 300, 300, 300]}"

  ros2 launch roadster_description roadster_sim.launch.py               # flat runway
  ros2 launch roadster_description roadster_sim.launch.py gui:=true
  ros2 launch roadster_description roadster_sim.launch.py \\
      world:=roadster_zero_g_test z:=10 paused:=true

World lookup: <world>.world.sdf in roadster_description/worlds, then
urban_gazebo/worlds. ~/vlm_av_ws/maps/<world>.json, when it exists, owns the
world's geometry (one owner: scripts/generate_stunt_world.py): it supplies the
fan's analytic surfaces and the default spawn pose. Without it the actuation
system falls back to a z = 0 ground plane and the car spawns at the origin.
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_world(world):
    for pkg in ('roadster_description', 'urban_gazebo'):
        try:
            path = os.path.join(get_package_share_directory(pkg), 'worlds', f'{world}.world.sdf')
        except Exception:  # package not installed
            continue
        if os.path.isfile(path):
            return path
    raise RuntimeError(f'world {world}.world.sdf not found in roadster_description or urban_gazebo')


def setup(context):
    cfg = context.launch_configurations
    pkg = get_package_share_directory('roadster_description')
    world = cfg['world']
    world_file = find_world(world)
    urdf_file = os.path.join(pkg, 'urdf', 'roadster.urdf.xacro')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge_roadster.yaml')

    # maps/<world>.json (generate_stunt_world.py) owns the world's geometry:
    # the fan's analytic surfaces and the default spawn pose
    map_json = os.path.expanduser(f'~/vlm_av_ws/maps/{world}.json')
    world_map = {}
    if os.path.isfile(map_json):
        with open(map_json) as f:
            world_map = json.load(f)
    surfaces = cfg['surfaces_file']
    if surfaces == 'auto':
        surfaces = map_json if 'surfaces' in world_map else ''
    spawn_default = {'x': 0.0, 'y': 0.0, 'z': 0.05, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    spawn_default.update(world_map.get('spawn', {}))

    xacro_args = {k: cfg[k] for k in ('chassis_mass', 'tire_mu', 'with_camera',
                                      'fan_initial_speed')}
    xacro_args['surfaces_file'] = surfaces
    cmd = ['xacro ', urdf_file]
    for k, v in xacro_args.items():
        cmd += [f' {k}:=', v]
    robot_description = ParameterValue(Command(cmd), value_type=str)

    # server-only unless gui:=true; NEVER --headless-rendering (WSL freeze).
    # paused:=true leaves the world paused; the test/mission node unpauses it
    # through /world/<world>/control once it is ready.
    gz_cmd = ['ign', 'gazebo', world_file]
    if cfg['paused'].lower() != 'true':
        gz_cmd.insert(2, '-r')
    if cfg['gui'].lower() != 'true':
        gz_cmd.insert(2, '-s')

    spawner = lambda name: Node(  # noqa: E731
        package='controller_manager', executable='spawner',
        arguments=[name, '--controller-manager', '/controller_manager'],
        output='screen')

    # gz_ros2_control's system plugin lives in /opt/ros/humble/lib, not on
    # Gazebo's default search path; roadster_gz_plugins adds its own lib dir
    # through its environment hook (already in the inherited variable)
    plugin_path = os.pathsep.join(filter(None, [
        os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'humble'), 'lib'),
        os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')]))

    # Fortress's contact system ignores the sensor's <topic> and publishes on
    # the scoped default name, which embeds the world - bridged as a separate
    # positional-arg node (a positional arg on the config_file node silently
    # drops the config_file bridges, see urban_world.launch.py)
    contact_gz = (f'/world/{world}/model/roadster/link/base_link/sensor/'
                  'body_contact/contact')

    pose = []
    for flag, key in (('-x', 'x'), ('-y', 'y'), ('-z', 'z'),
                      ('-R', 'roll'), ('-P', 'pitch'), ('-Y', 'yaw')):
        value = cfg[key] if cfg[key] != 'auto' else str(spawn_default[key])
        pose += [flag, value]

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

        TimerAction(period=5.0, actions=[
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'roadster', '-world', world,
                            '-topic', 'robot_description'] + pose,
                 output='screen')]),

        # the controller_manager lives inside Gazebo (gz_ros2_control), so the
        # spawners must wait for the model. NOTE: gz_ros2_control only comes up
        # once the world steps, so with paused:=true the spawners wait until
        # the world is unpaused.
        TimerAction(period=9.0, actions=[
            spawner('joint_state_broadcaster'),
            spawner('wheel_effort_controller'),
            spawner('steering_position_controller')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='roadster_flat_test'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        # spawn pose: 'auto' = the world JSON's "spawn", else base_link (which
        # sits at ground level, same as the sedan) 5 cm above the origin
        *[DeclareLaunchArgument(k, default_value='auto')
          for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')],
        DeclareLaunchArgument('chassis_mass', default_value='1730'),
        DeclareLaunchArgument('tire_mu', default_value='1.2'),
        DeclareLaunchArgument('with_camera', default_value='false'),
        DeclareLaunchArgument('fan_initial_speed', default_value='0.0',
                              description='fan spin state at spawn (1.0 for a ceiling hold)'),
        DeclareLaunchArgument('surfaces_file', default_value='auto',
                              description="'auto' = ~/vlm_av_ws/maps/<world>.json if present"),
        OpaqueFunction(function=setup),
    ])
