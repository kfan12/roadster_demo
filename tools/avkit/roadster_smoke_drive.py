#!/usr/bin/env python3
"""roadster_smoke_drive.py - Phase 1 smoke test for roadster_description.

Run against `ros2 launch roadster_description roadster_smoke.launch.py`.
1. At rest: sums the four wheel force/torque magnitudes and compares them with
   the sprung weight (everything above the wheel joints).
2. Tire-only launch: commands the same torque on all four wheels and reports
   the 0-60 mph time from /odom_truth, plus peak acceleration, wheel slip and
   yaw drift.
3. Reports the real-time factor (sim time / wall time) over the run.

  python3 tools/avkit/roadster_smoke_drive.py --torque 1500 --sprung-mass 310.2
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

MPH60 = 26.8224  # m/s
WHEELS = ('fl', 'fr', 'rl', 'rr')


class Smoke(Node):
    def __init__(self):
        super().__init__('roadster_smoke_drive')
        self.odom = None
        self.wrench = {}
        self.joints = None
        self.create_subscription(Odometry, '/odom_truth', self._odom, 50)
        for w in WHEELS:
            self.create_subscription(Wrench, f'/roadster/wheel_wrench/{w}',
                                     lambda m, w=w: self.wrench.__setitem__(w, m), 50)
        self.create_subscription(JointState, '/joint_states', self._js, 50)
        self.pub_eff = self.create_publisher(
            Float64MultiArray, '/wheel_effort_controller/commands', 10)
        self.pub_steer = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)

    def _odom(self, m):
        self.odom = m

    def _js(self, m):
        self.joints = m

    def stamp(self):
        s = self.odom.header.stamp
        return s.sec + s.nanosec * 1e-9

    def speed(self):
        v = self.odom.twist.twist.linear  # child (body) frame
        return math.hypot(v.x, v.y)

    def yaw(self):
        q = self.odom.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def effort(self, tau):
        self.pub_eff.publish(Float64MultiArray(data=[float(tau)] * 4))

    def spin_for(self, wall_s):
        end = time.monotonic() + wall_s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--torque', type=float, default=1500.0, help='per-wheel torque [N.m]')
    ap.add_argument('--sprung-mass', type=float, required=True,
                    help='mass above the wheel joints [kg] (chassis + steer links + sensors)')
    ap.add_argument('--timeout', type=float, default=60.0, help='wall-clock limit [s]')
    args = ap.parse_args()

    rclpy.init()
    n = Smoke()
    t0 = time.monotonic()
    while (n.odom is None or len(n.wrench) < 4 or n.joints is None) and time.monotonic() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.odom is None or len(n.wrench) < 4 or n.joints is None:
        print(f'FAIL: missing inputs odom={n.odom is not None} wrench={sorted(n.wrench)} '
              f'joint_states={n.joints is not None}')
        return 1
    print(f'joint_states names: {list(n.joints.name)}')

    # settle at rest with zero torque and straight wheels
    n.pub_steer.publish(Float64MultiArray(data=[0.0, 0.0]))
    n.effort(0.0)
    n.spin_for(3.0)
    mags = {w: math.sqrt(n.wrench[w].force.x ** 2 + n.wrench[w].force.y ** 2 +
                         n.wrench[w].force.z ** 2) for w in WHEELS}
    total = sum(mags.values())
    expect = args.sprung_mass * 9.81
    print('at rest  |F| per wheel [N]: ' + '  '.join(f'{w}={mags[w]:.0f}' for w in WHEELS))
    print(f'at rest  sum |F| = {total:.0f} N, sprung weight = {expect:.0f} N, '
          f'error = {100 * (total - expect) / expect:+.1f}%')
    z_rest = n.odom.pose.pose.position.z
    print(f'at rest  base_link z = {z_rest:.4f} m')

    # steering: position step to +0.3 rad and back, read the joint angles
    steer_names = ('front_left_wheel_steering_joint', 'front_right_wheel_steering_joint')
    for target in (0.3, 0.0):
        n.pub_steer.publish(Float64MultiArray(data=[target, target]))
        t_cmd = n.stamp()
        reached = None
        while n.stamp() - t_cmd < 2.0:
            rclpy.spin_once(n, timeout_sec=0.01)
            pos = [n.joints.position[n.joints.name.index(j)] for j in steer_names]
            if reached is None and all(abs(p - target) < 0.01 for p in pos):
                reached = n.stamp() - t_cmd
        print(f'steering cmd {target:+.2f} rad -> left {pos[0]:+.3f}, right {pos[1]:+.3f} rad, '
              f'within 0.01 rad after {reached if reached is None else round(reached, 3)} s sim')

    # tire-only launch
    yaw0 = n.yaw()
    x0 = n.odom.pose.pose.position.x
    sim0, wall0 = n.stamp(), time.monotonic()
    v_prev, t_prev, a_peak = 0.0, sim0, 0.0
    t60 = None
    max_wheel_surface = 0.0
    while time.monotonic() - wall0 < args.timeout:
        n.effort(args.torque)
        rclpy.spin_once(n, timeout_sec=0.005)
        t = n.stamp()
        v = n.speed()
        if t - t_prev >= 0.05:
            a_peak = max(a_peak, (v - v_prev) / (t - t_prev))
            v_prev, t_prev = v, t
        if n.joints.velocity:
            max_wheel_surface = max(max_wheel_surface, max(abs(x) for x in n.joints.velocity) * 0.33)
        if v >= MPH60 and t60 is None:
            t60 = t - sim0
            sim_end, wall_end = t, time.monotonic()
            break
    n.effort(0.0)
    for _ in range(20):
        n.effort(0.0)
        rclpy.spin_once(n, timeout_sec=0.01)

    if t60 is None:
        print(f'FAIL: did not reach 60 mph within {args.timeout:.0f} s wall; '
              f'v = {n.speed():.2f} m/s after {n.stamp() - sim0:.2f} s sim')
        return 1
    dist = n.odom.pose.pose.position.x - x0
    print(f'launch   0-60 mph = {t60:.3f} s sim  (mean a = {MPH60 / t60:.2f} m/s^2 = '
          f'{MPH60 / t60 / 9.81:.2f} g), peak a (50 ms) = {a_peak:.2f} m/s^2, distance = {dist:.1f} m')
    print(f'launch   max wheel surface speed = {max_wheel_surface:.1f} m/s vs body {MPH60:.1f} m/s '
          f'(slip ratio {(max_wheel_surface - MPH60) / MPH60:+.2f})')
    print(f'launch   yaw drift = {math.degrees(n.yaw() - yaw0):+.2f} deg')
    print(f'RTF      = {(sim_end - sim0) / (wall_end - wall0):.2f}')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
