#!/usr/bin/env python3
"""stunt_viz.py - RViz markers for the roadster stunt demos (visual only).

Publishes:
  /stunt/viz_world  MarkerArray (transient local): the world's stunt surfaces
                    (ramp arc, overhang, ceilings) drawn from maps/<world>.json,
                    the same analytic geometry the fan and the mission use
  /stunt/viz        MarkerArray, 20 Hz: one arrow per cold-gas nozzle (in its
                    TF frame, pointing along the exhaust, length ~ thrust), the
                    fan suction arrow, and a text panel with the mission state
  /stunt/viz_flow   MarkerArray, 20 Hz: the air itself - exhaust puffs launched
                    from every firing nozzle (white) and from the fan exhaust
                    (blue) in the world frame, growing and fading over ~0.6 s
                    so they trail behind the car through the flip; a fan disc
                    under the floor glowing with fan speed, and animated
                    suction streaks converging from the surface into it
  /stunt/viz (also) IMU: specific-force (green) and angular-rate (magenta)
                    arrows at the IMU + a readout with the estimate error;
                    control: requested (red) vs allocated (yellow) thrust
                    force and requested torque (cyan) at the COM, per-wheel
                    drive/brake torque arrows, and a control panel
  /stunt/trail      Path of base_link in the odom (= world) frame (truth)
  /stunt/trail_est  the same from /odom_est (IMU state estimate)

Nothing here feeds back into control.
"""
import json
import math
import random

import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from roadster_msgs.msg import StuntStatus
from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import Float32Array
from std_msgs.msg import ColorRGBA, Float64
from visualization_msgs.msg import Marker, MarkerArray

G = 9.81
CLAMP = 60.0  # draw at most this far from the origin (the ground plane is infinite)
CHASSIS_Z = 0.33  # chassis frame above base_link (nozzles.yaml is in the chassis frame)
FAN_MG = 1.5      # fan f_max as a multiple of m g (roadster.urdf.xacro)
# fan exhaust vents on the rear deck (chassis frame) and their blow direction,
# same as the Gazebo fx_fan_* emitters
FAN_VENTS = [((-1.8, 0.5, 0.7), (-0.45, 0.0, 0.89)), ((-1.8, -0.5, 0.7), (-0.45, 0.0, 0.89))]
PUFF_LIFE = 0.6   # [s]
AGE_BINS = (0.15, 0.35, PUFF_LIFE)  # puffs drawn in three sizes as they grow
AGE_SIZE = (0.25, 0.5, 0.85)        # [m]


def quat_rot(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 q_vec x v ; v' = v + w t + q_vec x t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def pt(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


class StuntViz(Node):
    def __init__(self):
        super().__init__('stunt_viz')
        world_json = self.declare_parameter('world_json', '').value
        nozzles_file = self.declare_parameter('nozzles_file', '').value
        self.mass = self.declare_parameter('mass', 1800.0).value
        # arrow length per m g of force [m]
        self.arrow_per_mg = self.declare_parameter('arrow_per_mg', 1.2).value
        # HUD column (world frame): in the sky left of demo_a.rviz's side-on view
        self.hud_x = self.declare_parameter('hud_x', -24.0).value
        self.hud_top = self.declare_parameter('hud_top', 21.0).value
        self.hud_size = self.declare_parameter('hud_size', 0.55).value
        self.hud_gap = self.declare_parameter('hud_gap', 0.8).value
        geo = yaml.safe_load(open(nozzles_file))['nozzles']
        self.nozzles = sorted(geo)
        self.nozzle_geo = [(geo[n]['xyz'], [-c for c in geo[n]['force_dir']], geo[n]['f_max_mg'])
                           for n in self.nozzles]  # exit point, exhaust direction, f_max
        self.thrust = [0.0] * len(self.nozzles)
        self.puffs = []  # [x, y, z, vx, vy, vz, age, kind]
        self.prev_odom = None
        self.car_vel = (0.0, 0.0, 0.0)
        self.rng = random.Random(0)
        self.fan = 0.0
        self.status = None
        self.odom = None
        self.trail = Path()
        self.trail.header.frame_id = 'odom'
        self.last_trail_t = -1.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_world = self.create_publisher(MarkerArray, '/stunt/viz_world', latched)
        self.pub = self.create_publisher(MarkerArray, '/stunt/viz', 10)
        self.pub_flow = self.create_publisher(MarkerArray, '/stunt/viz_flow', 10)
        self.pub_trail = self.create_publisher(Path, '/stunt/trail', 10)
        self.create_subscription(Float32Array, '/roadster/thruster_state', self._thrust, 10)
        self.create_subscription(Float64, '/roadster/fan_force',
                                 lambda m: setattr(self, 'fan', m.data), 10)
        self.create_subscription(StuntStatus, '/stunt/status',
                                 lambda m: setattr(self, 'status', m), 10)
        self.create_subscription(Odometry, '/odom_truth', self._odom, 50)
        # IMU + the IMU state estimate (its path is drawn next to the true one)
        self.imu = None
        self.odom_est = None
        self.trail_est = Path()
        self.trail_est.header.frame_id = 'odom'
        self.last_est_t = -1.0
        self.pub_trail_est = self.create_publisher(Path, '/stunt/trail_est', 10)
        self.create_subscription(Imu, '/imu', lambda m: setattr(self, 'imu', m), 10)
        self.create_subscription(Odometry, '/odom_est', self._odom_est, 50)

        self.pub_world.publish(self.world_markers(world_json))
        self.create_timer(0.05, self.tick)

    # ------------------------------------------------------------ world
    def world_markers(self, path):
        arr = MarkerArray()
        try:
            surfaces = json.load(open(path))['surfaces']
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().warn(f'no surfaces to draw from {path!r}: {e}')
            return arr
        for i, s in enumerate(surfaces):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns = 'world'
            m.id = i
            m.type = Marker.TRIANGLE_LIST
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.pose.orientation.w = 1.0
            if s['type'] == 'plane':
                n = s['normal']
                if abs(n[2]) < 0.99 or s['name'] == 'ground':
                    continue  # the grid stands in for the ground; only flat slabs drawn
                lo = [max(v, -CLAMP) for v in s['box_min']]
                hi = [min(v, CLAMP) for v in s['box_max']]
                z = s['point'][2]
                quad = [pt(lo[0], lo[1], z), pt(hi[0], lo[1], z), pt(hi[0], hi[1], z),
                        pt(lo[0], hi[1], z)]
                m.points = [quad[0], quad[1], quad[2], quad[0], quad[2], quad[3]]
                m.color = rgba(0.75, 0.55, 0.45, 0.85)
            elif s['type'] == 'arc_y':
                cx, cz = s['center_xz']
                r, (y0, y1) = s['radius'], s['y_range']
                n = max(8, int(math.degrees(s['theta1'] - s['theta0']) / 2))
                for k in range(n):
                    a0 = s['theta0'] + (s['theta1'] - s['theta0']) * k / n
                    a1 = s['theta0'] + (s['theta1'] - s['theta0']) * (k + 1) / n
                    p = [(cx + r * math.cos(a), cz + r * math.sin(a)) for a in (a0, a1)]
                    q = [pt(p[0][0], y0, p[0][1]), pt(p[1][0], y0, p[1][1]),
                         pt(p[1][0], y1, p[1][1]), pt(p[0][0], y1, p[0][1])]
                    m.points += [q[0], q[1], q[2], q[0], q[2], q[3]]
                m.color = rgba(0.6, 0.6, 0.68, 0.85)
            else:
                continue
            arr.markers.append(m)
        return arr

    # ------------------------------------------------------------ live
    def _thrust(self, m):
        if len(m.data) == len(self.thrust):
            self.thrust = list(m.data)

    def _odom(self, m):
        self.odom = m
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        if self.prev_odom is not None and t - self.prev_odom[0] > 1e-4:
            t0, p0 = self.prev_odom
            dt = t - t0
            self.car_vel = ((p.x - p0[0]) / dt, (p.y - p0[1]) / dt, (p.z - p0[2]) / dt)
        self.prev_odom = (t, (p.x, p.y, p.z))
        if t - self.last_trail_t >= 0.05:
            self.last_trail_t = t
            ps = PoseStamped()
            ps.header = m.header
            ps.pose = m.pose.pose
            self.trail.poses.append(ps)
            self.trail.header.stamp = m.header.stamp

    def _odom_est(self, m):
        self.odom_est = m
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - self.last_est_t >= 0.05:
            self.last_est_t = t
            ps = PoseStamped()
            ps.header = m.header
            ps.pose = m.pose.pose
            self.trail_est.poses.append(ps)
            self.trail_est.header.stamp = m.header.stamp

    def arrow(self, mid, frame, tip, color, shaft=0.18):
        a = Marker()
        a.header.frame_id = frame
        a.ns = 'thrust'
        a.id = mid
        a.type = Marker.ARROW
        a.pose.orientation.w = 1.0
        a.points = [pt(0, 0, 0), tip]
        a.scale.x, a.scale.y, a.scale.z = shaft, shaft * 2.0, shaft * 2.0
        a.color = color
        return a

    def tick(self):
        arr = MarkerArray()
        mg = self.mass * G
        for i, (name, f) in enumerate(zip(self.nozzles, self.thrust)):
            length = self.arrow_per_mg * f / mg
            # nozzle frame x = force on the car, so the plume points along -x
            arr.markers.append(self.arrow(
                i, f'nozzle_{name}', pt(-length, 0, 0),
                rgba(1.0, 0.55, 0.1, 0.95 if length > 0.02 else 0.0)))
        fan_len = self.arrow_per_mg * self.fan / mg
        fan = self.arrow(100, 'chassis', pt(0, 0, -0.1 - fan_len),
                         rgba(0.2, 0.5, 1.0, 0.9 if fan_len > 0.02 else 0.0), shaft=0.25)
        fan.ns = 'fan'
        arr.markers.append(fan)
        if self.status is not None and self.odom is not None:
            s, p, q = self.status, self.odom.pose.pose.position, self.odom.pose.pose.orientation
            up = 1 - 2 * (q.x * q.x + q.y * q.y)  # world z of the body z axis
            txt = Marker()
            txt.header.frame_id = 'odom'
            txt.ns = 'status'
            txt.id = 0
            txt.type = Marker.TEXT_VIEW_FACING
            # only a short state label rides with the car; the details go to
            # the fixed HUD column (panels around the car overlapped)
            txt.pose.position = pt(p.x, p.y, p.z + (2.2 if up > -0.5 else -2.2))
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.7
            txt.color = rgba(1, 1, 1, 1)
            txt.text = s.state
            arr.markers.append(txt)
            panels = [('mission_panel', rgba(1, 1, 1, 1),
                       f'MISSION  {s.state}\n'
                       f'speed {s.speed:4.1f} m/s   height {max(s.height, 0):4.1f} m\n'
                       f'thrust {sum(self.thrust) / 1000:5.1f} kN   fan {self.fan / 1000:4.1f} kN\n'
                       f'flip {math.degrees(s.roll_progress):5.1f} deg   gas {s.tank_mass:5.1f} kg')]
            panels += self.imu_and_control(arr, s, p, q)
            self.hud(arr, panels)
        self.pub.publish(arr)
        self.pub_trail.publish(self.trail)
        self.pub_trail_est.publish(self.trail_est)
        self.pub_flow.publish(self.flow())

    # ------------------------------------------------------------ HUD
    def hud(self, arr, panels):
        """Stack text panels in a fixed column in the sky, left of the default
        side-on view, top-down by line count so they never overlap. (A true
        screen overlay needs ros-humble-rviz-2d-overlay-plugins.)"""
        size = self.hud_size
        line = size * 1.3
        top = self.hud_top
        for ns, color, body in panels:
            n = body.count('\n') + 1
            arr.markers.append(self.text(0, ns, pt(self.hud_x, 0.0, top - n * line / 2),
                                         body, color, size))
            top -= n * line + self.hud_gap

    # ------------------------------------------------------------ IMU + control
    def text(self, mid, ns, pos, body, color, size=0.45):
        m = Marker()
        m.header.frame_id = 'odom'
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.pose.position = pos
        m.pose.orientation.w = 1.0
        m.scale.z = size
        m.color = color
        m.text = body
        return m

    def imu_and_control(self, arr, s, p, q):
        """IMU and control arrows; returns their HUD panels."""
        mg = self.mass * G
        panels = []
        # ---- IMU (sensor frame = chassis axes at the chassis origin)
        if self.imu is not None:
            f = self.imu.linear_acceleration
            w = self.imu.angular_velocity
            fa = self.arrow(0, 'imu_link', pt(f.x * 0.15, f.y * 0.15, f.z * 0.15),
                            rgba(0.2, 1.0, 0.3, 0.95), shaft=0.12)
            fa.ns = 'imu_specific_force'
            arr.markers.append(fa)
            wa = self.arrow(0, 'imu_link', pt(w.x * 1.0, w.y * 1.0, w.z * 1.0),
                            rgba(1.0, 0.2, 1.0, 0.95), shaft=0.12)
            wa.ns = 'imu_rate'
            arr.markers.append(wa)
            fn = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
            est = ''
            if self.odom_est is not None:
                e = self.odom_est.pose.pose.position
                err = math.dist((e.x, e.y, e.z), (p.x, p.y, p.z))
                est = f'\nestimate error {err * 100:5.1f} cm'
            panels.append(('imu_panel', rgba(0.6, 1.0, 0.6, 1.0),
                           f'IMU  |f| {fn / G:4.2f} g\n'
                           f'f  ({f.x:+6.1f} {f.y:+6.1f} {f.z:+6.1f}) m/s2\n'
                           f'w  ({w.x:+5.2f} {w.y:+5.2f} {w.z:+5.2f}) rad/s{est}'))
        # ---- control: requested vs allocated thrust force and torque at the COM
        R = quat_rot((q.x, q.y, q.z, q.w), (0.0, 0.0, CHASSIS_Z))
        com = (p.x + R[0], p.y + R[1], p.z + R[2])
        wd, wa_ = s.wrench_des, s.wrench_alloc  # forces world axes, torques body axes

        def world_arrow(mid, ns, vec, scale, color, shaft):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns = ns
            m.id = mid
            m.type = Marker.ARROW
            m.pose.orientation.w = 1.0
            m.points = [pt(*com), pt(com[0] + vec[0] * scale, com[1] + vec[1] * scale,
                                     com[2] + vec[2] * scale)]
            m.scale.x, m.scale.y, m.scale.z = shaft, shaft * 2, shaft * 2
            on = math.sqrt(sum(v * v for v in vec)) * scale > 0.05
            m.color = color if on else rgba(0, 0, 0, 0)
            return m
        arr.markers.append(world_arrow(0, 'ctrl_force_des', wd[:3], self.arrow_per_mg / mg,
                                       rgba(1.0, 0.15, 0.15, 0.9), 0.2))
        arr.markers.append(world_arrow(0, 'ctrl_force_alloc', wa_[:3], self.arrow_per_mg / mg,
                                       rgba(1.0, 0.9, 0.1, 0.9), 0.12))
        tq = quat_rot((q.x, q.y, q.z, q.w), tuple(wd[3:]))  # body torque -> world axes
        arr.markers.append(world_arrow(0, 'ctrl_torque_des', tq, 1.0 / 10000.0,
                                       rgba(0.1, 0.9, 1.0, 0.9), 0.12))
        # per-wheel drive (+) / brake (-) torque along the car's x axis
        for i, (wx, wy) in enumerate(((1.35, 0.8), (1.35, -0.8), (-1.35, 0.8), (-1.35, -0.8))):
            m = self.arrow(i, 'chassis', pt(0, 0, 0), rgba(0, 0, 0, 0), shaft=0.1)
            m.ns = 'ctrl_wheel_torque'
            m.points = [pt(wx, wy, 0.0), pt(wx + s.wheel_torque[i] / 1000.0, wy, 0.0)]
            m.color = (rgba(0.2, 1.0, 0.2, 0.9) if s.wheel_torque[i] >= 0
                       else rgba(1.0, 0.3, 0.2, 0.9))
            if abs(s.wheel_torque[i]) < 20:
                m.color = rgba(0, 0, 0, 0)
            arr.markers.append(m)
        panels.insert(0, (
            'control_panel', rgba(1.0, 0.85, 0.5, 1.0),
            f'CONTROL  {s.state}  ({s.t_state:4.1f} s)\n'
            f'speed {s.speed:5.2f} / ref {s.speed_ref:5.2f} m/s\n'
            f'wheel torque {" ".join(f"{x:+5.0f}" for x in s.wheel_torque)} N.m\n'
            f'fan {s.fan_cmd:3.1f}   attitude err {math.degrees(s.attitude_error):5.2f} deg\n'
            f'h {max(s.height, 0):5.2f} m  vz {s.vz:+6.2f} m/s\n'
            f'thrust des ({wd[0] / 1000:+5.1f} {wd[1] / 1000:+5.1f} {wd[2] / 1000:+5.1f}) kN\n'
            f'torque des ({wd[3] / 1000:+5.1f} {wd[4] / 1000:+5.1f} {wd[5] / 1000:+5.1f}) kN.m\n'
            f'impulse {s.impulse_used / 1000:5.1f} kN.s'))
        return panels

    # ------------------------------------------------------------ air flow
    def spawn(self, origin_ch, direction_ch, level, speed, kind, q, base):
        """Launch puffs from a chassis-frame exit along a chassis-frame direction."""
        o = quat_rot(q, (origin_ch[0], origin_ch[1], origin_ch[2] + CHASSIS_Z))
        d = quat_rot(q, direction_ch)
        n = max(1, int(round(level * 4)))
        for _ in range(n):
            j = [self.rng.uniform(-0.12, 0.12) for _ in range(3)]
            s = speed * self.rng.uniform(0.8, 1.2)
            self.puffs.append([base[0] + o[0], base[1] + o[1], base[2] + o[2],
                               self.car_vel[0] + s * (d[0] + j[0]),
                               self.car_vel[1] + s * (d[1] + j[1]),
                               self.car_vel[2] + s * (d[2] + j[2]),
                               self.rng.uniform(0.0, 0.04), kind])

    def flow(self):
        arr = MarkerArray()
        if self.odom is None:
            return arr
        st = self.odom.header.stamp
        t = st.sec + st.nanosec * 1e-9
        dt = min(max(t - getattr(self, 'flow_t', t), 0.0), 0.2)
        self.flow_t = t
        mg = self.mass * G
        p, qq = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        q = (qq.x, qq.y, qq.z, qq.w)
        base = (p.x, p.y, p.z)

        # age and move the existing puffs (exhaust slows quickly in still air)
        drag = math.exp(-4.0 * dt)
        for pf in self.puffs:
            pf[3] *= drag
            pf[4] *= drag
            pf[5] *= drag
            pf[0] += pf[3] * dt
            pf[1] += pf[4] * dt
            pf[2] += pf[5] * dt
            pf[6] += dt
        self.puffs = [pf for pf in self.puffs if pf[6] < PUFF_LIFE and pf[2] > -0.2]
        if dt > 0:
            for (xyz, exhaust, k), f in zip(self.nozzle_geo, self.thrust):
                level = f / (k * mg) if k > 0 else 0.0
                if level > 0.02:
                    self.spawn(xyz, exhaust, level, 14.0 + 10.0 * level, 'gas', q, base)
            fan_level = self.fan / (FAN_MG * mg)
            if fan_level > 0.02:
                for xyz, d in FAN_VENTS:
                    self.spawn(xyz, d, fan_level, 6.0, 'fan', q, base)

        # puffs: one SPHERE_LIST per (kind, age bin) so they grow as they age
        for ki, (kind, rgb) in enumerate((('gas', (0.95, 0.95, 1.0)),
                                          ('fan', (0.45, 0.75, 1.0)))):
            lo = 0.0
            for bi, (hi, size) in enumerate(zip(AGE_BINS, AGE_SIZE)):
                m = Marker()
                m.header.frame_id = 'odom'
                m.ns = f'puffs_{kind}'
                m.id = bi
                m.type = Marker.SPHERE_LIST
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = size
                for pf in self.puffs:
                    if pf[7] == kind and lo <= pf[6] < hi:
                        m.points.append(pt(pf[0], pf[1], pf[2]))
                        m.colors.append(rgba(*rgb, 0.75 * (1.0 - pf[6] / PUFF_LIFE)))
                if not m.points:
                    m.action = Marker.DELETE
                arr.markers.append(m)
                lo = hi

        # fan: glowing disc under the floor + suction streaks converging into it
        fan_level = min(self.fan / (FAN_MG * mg), 1.0)
        disc = Marker()
        disc.header.frame_id = 'chassis'
        disc.ns = 'fan'
        disc.id = 0
        disc.type = Marker.CYLINDER
        disc.pose.position = pt(0, 0, -0.09)
        disc.pose.orientation.w = 1.0
        disc.scale.x = disc.scale.y = 1.3
        disc.scale.z = 0.03
        disc.color = rgba(0.2, 0.55, 1.0, 0.15 + 0.8 * fan_level)
        arr.markers.append(disc)
        streaks = Marker()
        streaks.header.frame_id = 'chassis'
        streaks.ns = 'fan'
        streaks.id = 1
        streaks.type = Marker.LINE_LIST
        streaks.pose.orientation.w = 1.0
        streaks.scale.x = 0.04
        streaks.color = rgba(0.35, 0.7, 1.0, 0.9 * fan_level)
        if fan_level > 0.02:
            phase = (t * 1.8) % 1.0
            for i in range(16):
                a = 2 * math.pi * i / 16
                for k in range(2):
                    s0 = (phase + 0.5 * k) % 1.0   # 1 = outer edge, 0 = disc
                    r0, r1 = 0.65 + 1.9 * (1 - s0), 0.65 + 1.9 * max(1 - s0 - 0.18, 0.0)
                    z0, z1 = -0.3 + 0.2 * s0, -0.3 + 0.2 * (s0 + 0.18)
                    streaks.points += [pt(r0 * math.cos(a) * 1.6, r0 * math.sin(a), z0),
                                       pt(r1 * math.cos(a) * 1.6, r1 * math.sin(a), z1)]
        else:
            streaks.action = Marker.DELETE
        arr.markers.append(streaks)
        return arr


def main():
    rclpy.init()
    try:
        rclpy.spin(StuntViz())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C / launch shutdown: exit quietly, not with a traceback


if __name__ == '__main__':
    main()
