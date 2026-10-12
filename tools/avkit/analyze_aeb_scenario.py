#!/usr/bin/env python3
"""analyze_aeb_scenario.py - metrics for one recorded AEB scenario (Day 18 shape, shadow mode).

  python3 tools/avkit/analyze_aeb_scenario.py latest [--plots]

No ROS, no pandas (numpy; matplotlib only behind --plots). Reads a session
dir from record_aeb_scenario.py and writes metrics.json + report.md (+ plots
.png) into it.

Ground truth: meta.json carries the map's road_block pose; the true gap from
the sedan's front bumper to the block's near face is

    gt_standoff = (block - ego_xy) . block_heading - block_depth/2 - BUMPER_X

with ego_xy from /odom_truth (absolute world frame, same frame as the map).
BUMPER_X = ch_l/2 = 2.25 m (sedan.urdf.xacro; base_link is chassis centre).
This is what AEB's own obstacle_distance_m is measured against.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

BUMPER_X = 2.25           # sedan.urdf.xacro ch_l / 2
HALF_WIDTH = 0.9          # sedan.urdf.xacro ch_w / 2
EMERGENCY_DECEL = 6.0     # aeb_params.yaml ego_emergency_decel_mps2
RESPONSE_TIME = 0.3       # aeb_params.yaml response_time_sec
STATE_ORDER = ['INACTIVE', 'MONITORING', 'WARNING', 'BRAKING', 'STOP_HOLD', 'FAULT']


def load_jsonl(path):
    rows = []
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    r = json.loads(line)
                    if r.get('t', 1.0) > 0.0:   # t=0: recorded before /clock arrived
                        rows.append(r)
    return rows


def r3(x):
    return None if x is None else round(float(x), 3)


def interp(t_query, t, y):
    if len(t) == 0:
        return None
    return float(np.interp(t_query, t, y))


def gaps(rows, t0=None, t1=None):
    t = np.array([r['t'] for r in rows if (t0 is None or r['t'] >= t0) and (t1 is None or r['t'] <= t1)])
    if len(t) < 2:
        return None, None
    d = np.diff(t)
    return float(np.median(d)), float(d.max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session', help='session dir or "latest"')
    ap.add_argument('--sessions-root', default=os.path.expanduser('~/vlm_av_ws/data/aeb_scenarios'))
    ap.add_argument('--plots', action='store_true')
    args = ap.parse_args()

    sess = args.session
    if sess == 'latest':
        dirs = sorted(d for d in os.listdir(args.sessions_root)
                      if os.path.isdir(os.path.join(args.sessions_root, d)))
        if not dirs:
            sys.exit('no scenarios recorded yet')
        sess = os.path.join(args.sessions_root, dirs[-1])
    elif not os.path.isdir(sess):
        sess = os.path.join(args.sessions_root, sess)

    L = {n: load_jsonl(os.path.join(sess, n + '.jsonl'))
         for n in ('aeb_status', 'aeb_cmd', 'cmd_vel', 'odom_truth', 'odom_ekf',
                   'lane_path', 'plan_path', 'target_speed')}
    meta = {}
    if os.path.exists(os.path.join(sess, 'meta.json')):
        with open(os.path.join(sess, 'meta.json')) as f:
            meta = json.load(f)
    status = L['aeb_status']
    if not status:
        sys.exit('no aeb_status rows recorded')
    odom = L['odom_truth']

    m = {'session': os.path.basename(sess), 'world': meta.get('world'),
         'target_speed_mps': meta.get('target_speed_mps')}

    # ---------------- ground-truth standoff to the road block ---------------
    rb = meta.get('road_block')
    to = np.array([r['t'] for r in odom]) if odom else np.array([])
    vo = np.array([r['v'] for r in odom]) if odom else np.array([])
    gt = None
    if rb and odom:
        hx, hy = math.cos(rb['yaw']), math.sin(rb['yaw'])
        gt = np.array([(rb['x'] - r['x']) * hx + (rb['y'] - r['y']) * hy
                       - rb['size'][0] / 2.0 - BUMPER_X for r in odom])
        # standoff only means something while the block overlaps the car's
        # lateral extent; elsewhere (e.g. block in the oncoming lane) the car
        # can never reach it, so the along-road gap is masked to NaN.
        lat = np.array([-(rb['x'] - r['x']) * hy + (rb['y'] - r['y']) * hx for r in odom])
        in_path = np.abs(lat) < rb['size'][1] / 2.0 + HALF_WIDTH
        gt = np.where(in_path, gt, np.nan)
        m['road_block'] = rb
        m['block_in_ego_path'] = bool(in_path.any())
        m['min_lateral_offset_to_block_m'] = r3(np.abs(lat).min())
        if not in_path.any():
            gt = None
    if gt is not None:
        m['min_gt_standoff_m'] = r3(np.nanmin(gt))
        # contact: bumper reaches the block face (2 cm tolerance), else the
        # car stalls within 0.3 m of it (collision geometry stops it).
        contact = np.where(np.nan_to_num(gt, nan=1e9) <= 0.02)[0]
        if len(contact) == 0:
            contact = np.where((np.nan_to_num(gt, nan=1e9) <= 0.3) & (vo < 0.05))[0]
        m['contact'] = bool(len(contact))
        if len(contact):
            i = int(contact[0])
            m['contact_t'] = r3(to[i])
            m['impact_speed_mps'] = r3(vo[max(0, i - 5):i + 1].max())

    # ---------------- AEB state timeline -------------------------------------
    ts = np.array([r['t'] for r in status])
    states = [r['state'] for r in status]
    timeline = []
    for i, s in enumerate(states):
        if i == 0 or s != states[i - 1]:
            timeline.append({'t': r3(ts[i]), 'state': s})
    m['state_timeline'] = timeline
    m['state_counts'] = {s: states.count(s) for s in STATE_ORDER if states.count(s)}
    med, mx = gaps(status)
    m['aeb_status_rate_hz'] = r3(1.0 / med) if med else None
    m['aeb_status_max_gap_s'] = r3(mx)
    m['fault_reasons'] = sorted({r['reason'] for r in status if r['state'] == 'FAULT'})

    def at(t):
        out = {'t': r3(t)}
        if len(to):
            out['truth_speed_mps'] = r3(interp(t, to, vo))
        if gt is not None:
            out['gt_standoff_m'] = r3(interp(t, to, gt))
        return out

    for name in ('WARNING', 'BRAKING', 'STOP_HOLD'):
        idx = next((i for i, s in enumerate(states) if s == name), None)
        if idx is None:
            continue
        r = status[idx]
        d = at(r['t'])
        d.update({'perceived_distance_m': r['obstacle_distance_m'], 'ttc_sec': r['ttc_sec'],
                  'd_safe_m': r['d_safe_m'], 'aeb_speed_mps': r['relative_speed_mps']})
        if name == 'BRAKING' and d.get('gt_standoff_m') is not None:
            v = d['truth_speed_mps']
            need = v * v / (2 * EMERGENCY_DECEL)
            d['stop_margin_m'] = r3(d['gt_standoff_m'] - need)
            d['stop_margin_with_response_m'] = r3(d['gt_standoff_m'] - need - v * RESPONSE_TIME)
            if m.get('contact_t') is not None:
                d['time_before_contact_s'] = r3(m['contact_t'] - r['t'])
        m['first_' + name.lower()] = d

    # first time the block was perceived at all (any in-corridor cluster)
    seen = [r for r in status if r['obstacle_distance_m'] >= 0]
    if seen:
        d = at(seen[0]['t'])
        d['perceived_distance_m'] = seen[0]['obstacle_distance_m']
        m['first_detection'] = d

    # ---------------- perception accuracy vs ground truth ---------------------
    if gt is not None:
        rows = [r for r in status if r['obstacle_distance_m'] >= 0]
        if rows:
            t_r = np.array([r['t'] for r in rows])
            gt_r = np.interp(t_r, to, gt)
            pd = np.array([r['obstacle_distance_m'] for r in rows])
            ok = gt_r > 0.05                 # before contact only
            err = pd[ok] - gt_r[ok]
            if len(err):
                m['distance_error_m'] = {'n': int(len(err)), 'mean': r3(err.mean()),
                                         'std': r3(err.std()), 'max_abs': r3(np.abs(err).max())}
                # fraction of ticks, while the block was 0-30 m ahead, in which
                # AEB saw anything in the corridor at all
                win = (np.interp(ts, to, gt) > 0.05) & (np.interp(ts, to, gt) < 30)
                if win.any():
                    det = np.array([r['obstacle_distance_m'] >= 0 for r in status])[win]
                    m['detection_rate_0_30m'] = r3(det.mean())
            # ticks reporting an obstacle much nearer than the block (phantom)
            m['phantom_ticks_gt_minus_3m'] = int(((gt_r - pd) > 3.0).sum())

    # false positives: WARNING/BRAKING with no in-corridor obstacle is a bug by
    # construction; in a world with no road block, any WARNING/BRAKING is one.
    risky = [r for r in status if r['state'] in ('WARNING', 'BRAKING')]
    # ENTERING BRAKING with nothing in the corridor is a bug by construction;
    # staying in BRAKING after the cluster vanished is the designed latch
    # (release only via STOP_HOLD), so it is counted separately.
    m['braking_entries_without_obstacle'] = sum(
        1 for i, r in enumerate(status)
        if r['state'] == 'BRAKING' and r['obstacle_distance_m'] < 0
        and (i == 0 or status[i - 1]['state'] != 'BRAKING'))
    m['latched_braking_ticks_without_obstacle'] = sum(
        1 for r in risky if r['state'] == 'BRAKING' and r['obstacle_distance_m'] < 0)
    if not rb:
        m['false_positive_warning_ticks'] = sum(1 for r in risky if r['state'] == 'WARNING')
        m['false_positive_braking_ticks'] = sum(1 for r in risky if r['state'] == 'BRAKING')
        m['false_positive_events'] = [{'t': r['t'], 'state': r['state'],
                                       'd': r['obstacle_distance_m'], 'ttc': r['ttc_sec']}
                                      for r in risky][:30]

    # ---------------- after contact / at standstill ---------------------------
    if m.get('contact_t') is not None:
        after = [r['state'] for r in status if r['t'] >= m['contact_t'] + 1.0]
        if after:
            m['states_after_contact'] = {s: after.count(s) for s in STATE_ORDER if after.count(s)}
            m['state_at_end'] = after[-1]

        # speed AEB believes (from /odom_ekf) vs truth, once the car is pinned
        # against the block: wheel slip makes wheel-odom/EKF speed meaningless
        if len(to):
            late = [r for r in status if r['t'] >= m['contact_t'] + 1.0]
            if late:
                m['after_contact_aeb_speed_mps_mean'] = r3(np.mean([r['relative_speed_mps'] for r in late]))
                m['after_contact_truth_speed_mps_mean'] = r3(np.mean(np.interp([r['t'] for r in late], to, vo)))

    # detection coverage by true range band (block ahead, before contact)
    if gt is not None:
        gts = np.interp(ts, to, gt)
        det = np.array([r['obstacle_distance_m'] >= 0 for r in status])
        bands = {}
        for lo, hi in ((0.05, 5), (5, 10), (10, 15), (15, 20), (20, 30)):
            w = (gts > lo) & (gts <= hi)
            if w.any():
                bands[f'{lo:g}-{hi:g}m'] = r3(det[w].mean())
        m['detection_rate_by_range'] = bands

    # ---------------- upstream health (confound check) ------------------------
    t_end = m.get('contact_t')
    for name in ('lane_path', 'plan_path'):
        med, mx = gaps(L[name], t1=t_end)
        m[name + '_rate_hz'] = r3(1.0 / med) if med else None
        m[name + '_max_gap_s'] = r3(mx)
    if odom:
        m['peak_truth_speed_mps'] = r3(vo.max())
    cmd = L['cmd_vel']
    if cmd and t_end is not None:
        pre = [c['v'] for c in cmd if c['t'] <= t_end]
        m['cmd_vel_at_contact_mps'] = r3(pre[-1]) if pre else None

    with open(os.path.join(sess, 'metrics.json'), 'w') as f:
        json.dump(m, f, indent=1)

    lines = ['# AEB scenario report - ' + m['session'], '']
    for k, v in m.items():
        lines.append(f'- **{k}**: {json.dumps(v)}')
    report = '\n'.join(lines) + '\n'
    with open(os.path.join(sess, 'report.md'), 'w') as f:
        f.write(report)
    print(report)

    if args.plots:
        plot(sess, m, L, status, to, vo, gt)


def plot(sess, m, L, status, to, vo, gt):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    ts = np.array([r['t'] for r in status])
    t0 = min([ts[0]] + ([to[0]] if len(to) else []))
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

    if len(to):
        ax[0].plot(to - t0, vo, label='truth speed', color='k')
    cmd = L['cmd_vel']
    if cmd:
        ax[0].plot([c['t'] - t0 for c in cmd], [c['v'] for c in cmd], label='/cmd_vel (MPC)', alpha=0.7)
    ac = L['aeb_cmd']
    if ac:
        ax[0].plot([c['t'] - t0 for c in ac], [c['v'] for c in ac], label='/control/aeb_cmd (shadow)',
                   alpha=0.7, ls='--')
    ax[0].set_ylabel('speed [m/s]')
    ax[0].legend(loc='upper right', fontsize=8)

    pd = np.array([r['obstacle_distance_m'] for r in status])
    pd_plot = np.where(pd >= 0, pd, np.nan)
    ax[1].plot(ts - t0, pd_plot, '.', ms=3, label='AEB obstacle_distance_m')
    ax[1].plot(ts - t0, [r['d_safe_m'] for r in status], label='d_safe', alpha=0.8)
    if gt is not None:
        ax[1].plot(to - t0, gt, color='k', label='ground-truth standoff')
    ax[1].set_ylabel('distance [m]')
    ax[1].set_ylim(-1, 35)
    ax[1].legend(loc='upper right', fontsize=8)

    ttc = np.array([r['ttc_sec'] for r in status])
    ax[2].plot(ts - t0, np.where(ttc < 900, ttc, np.nan), '.', ms=3, label='TTC')
    ax[2].axhline(2.5, ls=':', color='orange', label='warning_ttc')
    ax[2].axhline(1.5, ls=':', color='red', label='brake_ttc')
    ax[2].set_ylim(0, 12)
    ax[2].set_ylabel('TTC [s]')
    ax[2].legend(loc='upper right', fontsize=8)

    sidx = [STATE_ORDER.index(r['state']) if r['state'] in STATE_ORDER else -1 for r in status]
    ax[3].step(ts - t0, sidx, where='post')
    ax[3].set_yticks(range(len(STATE_ORDER)))
    ax[3].set_yticklabels(STATE_ORDER, fontsize=8)
    ax[3].set_xlabel('sim time [s]')
    if m.get('contact_t') is not None:
        for a in ax:
            a.axvline(m['contact_t'] - t0, color='r', alpha=0.4)
    fig.suptitle(m['session'])
    fig.tight_layout()
    out = os.path.join(sess, 'timeline.png')
    fig.savefig(out, dpi=110)
    print('wrote', out)


if __name__ == '__main__':
    main()
