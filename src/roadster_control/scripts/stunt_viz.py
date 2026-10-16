#!/usr/bin/env python3
"""stunt_viz.py - RViz markers for the roadster stunt demos (visual only).

Publishes:
  /stunt/viz_world  MarkerArray (transient local): the world's stunt surfaces
                    (ramp arc, overhang, ceilings) drawn from maps/<world>.json,
                    the same analytic geometry the fan and the mission use
  /stunt/viz        MarkerArray, 20 Hz: one arrow per cold-gas nozzle (in its
                    TF frame, pointing along the exhaust, length ~ thrust), the
                    fan suction arrow, and a text panel with the mission state
  /stunt/trail      Path of base_link in the odom (= world) frame

Nothing here feeds back into control.
"""
import json
import math

import rclpy
import yaml
from av_feature_msgs.msg import StuntStatus
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Float32Array
from std_msgs.msg import ColorRGBA, Float64
from visualization_msgs.msg import Marker, MarkerArray

G = 9.81
CLAMP = 60.0  # draw at most this far from the origin (the ground plane is infinite)


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
        self.nozzles = sorted(yaml.safe_load(open(nozzles_file))['nozzles'])
        self.thrust = [0.0] * len(self.nozzles)
        self.fan = 0.0
        self.status = None
        self.odom = None
        self.trail = Path()
        self.trail.header.frame_id = 'odom'
        self.last_trail_t = -1.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_world = self.create_publisher(MarkerArray, '/stunt/viz_world', latched)
        self.pub = self.create_publisher(MarkerArray, '/stunt/viz', 10)
        self.pub_trail = self.create_publisher(Path, '/stunt/trail', 10)
        self.create_subscription(Float32Array, '/roadster/thruster_state', self._thrust, 10)
        self.create_subscription(Float64, '/roadster/fan_force',
                                 lambda m: setattr(self, 'fan', m.data), 10)
        self.create_subscription(StuntStatus, '/stunt/status',
                                 lambda m: setattr(self, 'status', m), 10)
        self.create_subscription(Odometry, '/odom_truth', self._odom, 50)

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
        if t - self.last_trail_t >= 0.05:
            self.last_trail_t = t
            ps = PoseStamped()
            ps.header = m.header
            ps.pose = m.pose.pose
            self.trail.poses.append(ps)
            self.trail.header.stamp = m.header.stamp

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
            txt.pose.position = pt(p.x, p.y, p.z + (3.0 if up > -0.5 else -3.0))
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.6
            txt.color = rgba(1, 1, 1, 1)
            txt.text = (f'{s.state}\n'
                        f'speed {s.speed:4.1f} m/s   height {max(s.height, 0):4.1f} m\n'
                        f'thrust {sum(self.thrust) / 1000:5.1f} kN   fan {self.fan / 1000:4.1f} kN\n'
                        f'flip {math.degrees(s.roll_progress):5.1f} deg   '
                        f'gas {s.tank_mass:5.1f} kg')
            arr.markers.append(txt)
        self.pub.publish(arr)
        self.pub_trail.publish(self.trail)


def main():
    rclpy.init()
    rclpy.spin(StuntViz())


if __name__ == '__main__':
    main()
