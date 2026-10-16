#!/usr/bin/env python3
"""record_stunt_run.py - record one roadster stunt run to JSONL (plan section 9).

Unpauses the (paused) world so recording starts at sim t = 0, then records
/stunt/status, /odom_truth, the four wheel wrenches, /roadster/body_contact and
/roadster/thruster_state until the mission reaches LANDED or ABORT (+ --tail
seconds) or --timeout. One JSON object per line: {"t": sim time, "topic":
..., ...fields}. Analyze with analyze_stunt_run.py.

  python3 tools/avkit/record_stunt_run.py --world roadster_drop16 --out data/stunt_runs/<session>
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from av_feature_msgs.msg import StuntStatus
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts, Float32Array


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, out):
        super().__init__('record_stunt_run')
        self.f = open(out, 'w')
        self.t = 0.0
        self.state = None
        self.n = 0
        self.create_subscription(StuntStatus, '/stunt/status', self._status, 100)
        self.create_subscription(Odometry, '/odom_truth', self._odom, 100)
        self.create_subscription(Contacts, '/roadster/body_contact', self._contact, 100)
        self.create_subscription(Float32Array, '/roadster/thruster_state', self._thrust, 100)
        for w in ('fl', 'fr', 'rl', 'rr'):
            self.create_subscription(Wrench, f'/roadster/wheel_wrench/{w}',
                                     lambda m, w=w: self._wrench(w, m), 100)

    def write(self, rec):
        self.f.write(json.dumps(rec) + '\n')
        self.n += 1

    def _status(self, m):
        self.t = stamp(m.header)
        self.state = m.state
        self.write({'t': self.t, 'topic': 'status', 'state': m.state, 't_state': m.t_state,
                    'height': m.height, 'vz': m.vz, 'roll_progress': m.roll_progress,
                    'attitude_error': m.attitude_error, 'tank_mass': m.tank_mass,
                    'feed_ratio': m.feed_ratio, 'impulse_used': m.impulse_used,
                    'fan_cmd': m.fan_cmd, 'wrench_des': list(m.wrench_des),
                    'wrench_alloc': list(m.wrench_alloc)})

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.write({'t': stamp(m.header), 'topic': 'odom', 'p': [p.x, p.y, p.z],
                    'q': [q.x, q.y, q.z, q.w]})

    def _contact(self, m):
        if m.contacts:
            self.write({'t': self.t, 'topic': 'body_contact', 'n': len(m.contacts),
                        'with': sorted({c.collision2.name for c in m.contacts} |
                                       {c.collision1.name for c in m.contacts})})

    def _thrust(self, m):
        self.write({'t': self.t, 'topic': 'thrust', 'f': [round(x, 1) for x in m.data]})

    def _wrench(self, w, m):
        f = m.force
        self.write({'t': self.t, 'topic': 'wheel', 'w': w, 'f': [f.x, f.y, f.z]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--out', required=True, help='session directory')
    ap.add_argument('--timeout', type=float, default=20.0, help='[s sim]')
    ap.add_argument('--tail', type=float, default=2.0, help='record after LANDED/ABORT [s sim]')
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    rec = Recorder(out / 'run.jsonl')
    subprocess.run(['ign', 'service', '-s', f'/world/{args.world}/control',
                    '--reqtype', 'ignition.msgs.WorldControl',
                    '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000',
                    '--req', 'pause: false'], check=True, capture_output=True)
    wall0 = time.monotonic()
    t_end = None
    while time.monotonic() - wall0 < 30 + 10 * args.timeout:
        rclpy.spin_once(rec, timeout_sec=0.01)
        if rec.state in ('LANDED', 'ABORT') and t_end is None:
            t_end = rec.t + args.tail
        if (t_end is not None and rec.t >= t_end) or rec.t > args.timeout:
            break
    rec.f.close()
    print(f'recorded {rec.n} records to {out / "run.jsonl"}, final state {rec.state}, '
          f'sim t = {rec.t:.2f} s')
    rclpy.shutdown()
    return 0 if rec.state is not None else 1


if __name__ == '__main__':
    sys.exit(main())
