#!/usr/bin/env python3
"""capture_stunt_snapshots.py - the README's stunt snapshots, from a live run.

  python3 tools/avkit/capture_stunt_snapshots.py --demo ramp|hop|launch --out docs/snapshots

Run it alongside a scored run with the onboard camera on, in the run's ROS
domain (scripts/run_stunt_scenario.sh isolates each run in its own), e.g.
  STUNT_DOMAIN_ID=42 scripts/run_stunt_scenario.sh roadster_hop hop <session> -- with_camera:=true &
  ROS_DOMAIN_ID=42 python3 tools/avkit/capture_stunt_snapshots.py --demo hop
It watches /stunt/status and, at each of the demo's moments below, saves the
first camera frame stamped at or after it (sim time) as a PNG, then exits
once all are saved (or after --timeout wall seconds):

  ramp (Demo A)
  roadster_driveoff_trackside.png  DRIVE_OFF + 0.3 s: upside down, rear thrust on
  roadster_flip_trackside.png      FLIP + 0.3 s: on its side mid-flip
  roadster_landing_trackside.png   LAND_BRAKE, height <= 1.0 m: the flare
  roadster_flip_onboard.png        FLIP + 0.3 s, the car's front camera
  hop (Demo B)
  roadster_hop_liftoff_trackside.png  HOP_BURN + 0.25 s: the lift burn
  roadster_hop_flip_trackside.png     BALLISTIC + 0.8 s: upside down mid-roll, over the Semi
  roadster_hop_landing_trackside.png  LAND_BRAKE, height <= 1.0 m: the flare
  roadster_hop_flip_onboard.png       BALLISTIC + 0.8 s, the car's front camera
  launch (Demo C)
  roadster_launch_start_trackside.png  LAUNCH + 0.4 s: rear thrust on
  roadster_launch_60mph_trackside.png  ROLLOUT: 60 mph, thrust cut
  roadster_launch_onboard.png          LAUNCH + 0.5 s, the car's front camera

No cv2: the PNG is written with zlib, so it runs under any Python.
"""
import argparse
import struct
import sys
import time
import zlib
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from roadster_msgs.msg import StuntStatus
from sensor_msgs.msg import Image

TRACKSIDE = '/camera/trackside/image'
ONBOARD = '/camera/front/image_raw'
# demo -> name -> (camera topic, state, seconds into the state, max height or None)
DEMO_SHOTS = {
    'ramp': {
        'roadster_driveoff_trackside.png': (TRACKSIDE, 'DRIVE_OFF', 0.3, None),
        'roadster_flip_trackside.png': (TRACKSIDE, 'FLIP', 0.3, None),
        'roadster_landing_trackside.png': (TRACKSIDE, 'LAND_BRAKE', 0.0, 1.0),
        'roadster_flip_onboard.png': (ONBOARD, 'FLIP', 0.3, None),
    },
    'hop': {
        'roadster_hop_liftoff_trackside.png': (TRACKSIDE, 'HOP_BURN', 0.25, None),
        'roadster_hop_flip_trackside.png': (TRACKSIDE, 'BALLISTIC', 0.8, None),
        'roadster_hop_landing_trackside.png': (TRACKSIDE, 'LAND_BRAKE', 0.0, 1.0),
        'roadster_hop_flip_onboard.png': (ONBOARD, 'BALLISTIC', 0.8, None),
    },
    'launch': {
        'roadster_launch_start_trackside.png': (TRACKSIDE, 'LAUNCH', 0.4, None),
        'roadster_launch_60mph_trackside.png': (TRACKSIDE, 'ROLLOUT', 0.0, None),
        'roadster_launch_onboard.png': (ONBOARD, 'LAUNCH', 0.5, None),
    },
}
SHOTS = DEMO_SHOTS['ramp']


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def write_png(path, msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'unsupported image encoding {msg.encoding}')
    data = bytes(msg.data)
    rows = []
    for r in range(msg.height):
        row = data[r * msg.step:r * msg.step + msg.width * channels]
        if msg.encoding.startswith('bgr'):  # swap to RGB(A)
            px = bytearray(row)
            px[0::channels], px[2::channels] = row[2::channels], row[0::channels]
            row = bytes(px)
        rows.append(b'\x00' + row)

    def chunk(tag, body):
        return struct.pack('>I', len(body)) + tag + body + struct.pack(
            '>I', zlib.crc32(tag + body) & 0xffffffff)

    color = 2 if channels == 3 else 6
    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack(
        '>IIBBBBB', msg.width, msg.height, 8, color, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(b''.join(rows), 9)) + chunk(b'IEND', b'')
    Path(path).write_bytes(png)


class Capture(Node):
    def __init__(self, out):
        super().__init__('capture_stunt_snapshots')
        self.out = out
        self.trigger = {}   # name -> sim time from which to take the next frame
        self.saved = set()
        self.state, self.t_state0 = None, 0.0
        self.create_subscription(StuntStatus, '/stunt/status', self.on_status, 50)
        for topic in {s[0] for s in SHOTS.values()}:
            self.create_subscription(
                Image, topic, lambda m, t=topic: self.on_image(t, m), qos_profile_sensor_data)

    def on_status(self, m):
        t = stamp(m.header)
        if m.state != self.state:
            self.state, self.t_state0 = m.state, t
        for name, (_, state, dt, max_h) in SHOTS.items():
            if name in self.trigger or m.state != state or t - self.t_state0 < dt:
                continue
            if max_h is not None and m.height > max_h:
                continue
            self.trigger[name] = t
            self.get_logger().info(f'{name}: from t = {t:.3f} ({state})')

    def on_image(self, topic, m):
        for name, (cam, *_rest) in SHOTS.items():
            if cam != topic or name in self.saved or name not in self.trigger:
                continue
            if stamp(m.header) >= self.trigger[name]:
                write_png(self.out / name, m)
                self.saved.add(name)
                self.get_logger().info(f'saved {self.out / name} (t = {stamp(m.header):.3f})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--demo', choices=sorted(DEMO_SHOTS), default='ramp')
    ap.add_argument('--out', default='docs/snapshots')
    ap.add_argument('--timeout', type=float, default=600.0, help='wall seconds')
    args = ap.parse_args()
    global SHOTS
    SHOTS = DEMO_SHOTS[args.demo]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Capture(out)
    t0 = time.monotonic()
    while len(n.saved) < len(SHOTS) and time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    missing = sorted(set(SHOTS) - n.saved)
    if missing:
        print('missing: ' + ', '.join(missing))
    n.destroy_node()
    rclpy.shutdown()
    return 1 if missing else 0


if __name__ == '__main__':
    sys.exit(main())
