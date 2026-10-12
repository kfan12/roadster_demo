#!/usr/bin/env python3
"""extract_frames.py - dump camera RGB + depth frames from a rosbag2 to PNG.

  python3 tools/avkit/extract_frames.py BAG_DIR OUT_DIR [--session SESSION_DIR]
      [--times T1 T2 ...] [--every SEC]

Frames are picked by image header stamp (sim time, same clock as the AEB
recorder's "t"). With --session, every AEB state transition in that session's
aeb_status.jsonl is added to --times, and each PNG is captioned with the AEB
state / perceived distance / TTC at that instant.

No cv_bridge (project rule, scripts/check_no_cv_bridge.sh): images are decoded
with raw numpy from sensor_msgs/Image.data, the same way
vlm_planner_py/img_convert.py does. Run with PYTHONNOUSERSITE=1 so the system
numpy 1.x (which rosbag2_py/matplotlib are built against) is used.
"""
import argparse
import bisect
import json
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

RGB_TOPIC = '/camera/front/image_raw'
DEPTH_TOPIC = '/camera/front/depth/image_raw'
LANE_DBG_TOPIC = '/lane/debug_image'


def image_to_numpy(m):
    if m.encoding in ('rgb8', 'bgr8'):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
        a = a.reshape(m.height, m.width, 3)
        return a[:, :, ::-1] if m.encoding == 'bgr8' else a
    if m.encoding == '32FC1':
        a = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
        return a
    raise ValueError(f'unsupported encoding {m.encoding}')


def read_images(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    wanted = {RGB_TOPIC, DEPTH_TOPIC, LANE_DBG_TOPIC}
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    out = {t: [] for t in wanted}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, Image)
        out[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m))
    for t in out:
        out[t].sort(key=lambda p: p[0])
    return out


def nearest(seq, t):
    if not seq:
        return None
    ts = [p[0] for p in seq]
    i = bisect.bisect_left(ts, t)
    cands = [j for j in (i - 1, i) if 0 <= j < len(seq)]
    j = min(cands, key=lambda k: abs(ts[k] - t))
    return seq[j]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--session', default=None)
    ap.add_argument('--times', type=float, nargs='*', default=[])
    ap.add_argument('--every', type=float, default=None)
    ap.add_argument('--max-depth', type=float, default=40.0)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    imgs = read_images(args.bag)
    print({k: len(v) for k, v in imgs.items()})
    os.makedirs(args.out, exist_ok=True)

    status = []
    times = list(args.times)
    if args.session:
        with open(os.path.join(args.session, 'aeb_status.jsonl')) as f:
            status = [json.loads(line) for line in f if line.strip()]
        for i, r in enumerate(status):
            if i == 0 or r['state'] != status[i - 1]['state']:
                times.append(r['t'])
    if args.every and imgs[RGB_TOPIC]:
        t = imgs[RGB_TOPIC][0][0]
        while t <= imgs[RGB_TOPIC][-1][0]:
            times.append(t)
            t += args.every
    st_ts = [r['t'] for r in status]

    for t in sorted(set(round(x, 2) for x in times)):
        rgb = nearest(imgs[RGB_TOPIC], t)
        dep = nearest(imgs[DEPTH_TOPIC], t)
        lane = nearest(imgs[LANE_DBG_TOPIC], t)
        panels = [(p, name) for p, name in ((rgb, 'rgb'), (dep, 'depth'), (lane, 'lane debug')) if p]
        if not panels:
            continue
        fig, ax = plt.subplots(1, len(panels), figsize=(4.6 * len(panels), 3.6))
        ax = np.atleast_1d(ax)
        for a, ((ts, msg), name) in zip(ax, panels):
            arr = image_to_numpy(msg)
            if arr.ndim == 2:
                d = np.where(np.isfinite(arr), arr, np.nan)
                im = a.imshow(np.clip(d, 0, args.max_depth), cmap='turbo_r', vmin=0, vmax=args.max_depth)
                fig.colorbar(im, ax=a, fraction=0.035, label='m')
            else:
                a.imshow(arr)
            a.set_title(f'{name} @ {ts:.2f}', fontsize=8)
            a.axis('off')
        cap = f't={t:.2f}'
        if st_ts:
            i = max(0, bisect.bisect_right(st_ts, t) - 1)
            r = status[i]
            cap += (f"  AEB {r['state']}  d={r['obstacle_distance_m']:.2f} m  "
                    f"ttc={min(r['ttc_sec'], 999):.2f} s  v={r['relative_speed_mps']:.2f} m/s")
        fig.suptitle(cap, fontsize=9)
        fig.tight_layout()
        path = os.path.join(args.out, f'frame_{t:010.2f}.png')
        fig.savefig(path, dpi=90)
        plt.close(fig)
        print('wrote', path)


if __name__ == '__main__':
    main()
