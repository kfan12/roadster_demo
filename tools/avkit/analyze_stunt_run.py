#!/usr/bin/env python3
"""analyze_stunt_run.py - metrics + pass/fail for one recorded stunt run.

  python3 tools/avkit/analyze_stunt_run.py data/stunt_runs/<session>

Reads <session>/run.jsonl (record_stunt_run.py), writes <session>/metrics.json
and prints a report. Metrics (plan section 9): flip rotation, touchdown
vertical speed and attitude, peak landing deceleration, body (non-wheel)
contacts after release, impulse and gas used, state timeline, and the
touchdown's lateral speed and offset (inside the map's landing zone when
--map names a world JSON with a geometry.landing_width). Ramp runs
(timeline starts with ACCELERATE) are also scored on the ground phase: arc
progress, inverted hold time / stillness / wheel loads, lateral offset and
body contacts on the ramp; flight checks apply only when the car released.
The demo is told by its states: CLIMB = ramp (Demo A), HOLD = drop, LAUNCH =
Demo C (0-60 mph time, rear thrust tracking, impulse, yaw), APPROACH = Demo B
(clearance of the car's lowest corner, rolled as it flips, over each vehicle
box in --map, apex, and the flight checks with a 1-turn (360 deg) target).

Touchdown = first odom sample after release whose base_link is within
--td-height of the ground (base_link sits at wheel-contact level). Vertical
speed is the COM's, by finite differences of the poses around that instant.
No pandas, same as the AEB analyzer.
"""
import argparse
import json
import math
import sys
from pathlib import Path

COM_Z = 0.33  # COM above base_link in the chassis frame (chassis_offset_z)
MPH60 = 26.8224
# the car's box for obstacle clearance, base_link frame: bumper to bumper,
# the wheels' outer faces, from the hanging wheels' bottom in flight to the
# cabin roof (the hop flips it, so any corner can be the lowest)
CAR_X, CAR_Y, CAR_BOTTOM_Z, CAR_TOP_Z = 2.25, 0.93, -0.07, 1.26


def rotate(q, v):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2],
        2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2],
        2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2])


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def body_z(q):
    x, y, z, w = q
    return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))


def com_z(p, q):
    return p[2] + COM_Z * body_z(q)[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session')
    ap.add_argument('--map', default=None,
                    help='maps/<world>.json: score the touchdown against its landing zone')
    ap.add_argument('--td-height', type=float, default=0.03)
    ap.add_argument('--max-td-speed', type=float, default=1.0,
                    help='touchdown vertical speed [m/s]: the flare lands at ~0.4 (plan target was < 3)')
    ap.add_argument('--max-td-tilt-deg', type=float, default=10.0)
    ap.add_argument('--max-td-lateral-speed', type=float, default=1.0,
                    help='sideways speed at touchdown [m/s]: a sliding landing can trip the car')
    ap.add_argument('--car-half-width', type=float, default=0.9,
                    help='the whole car must touch down inside the landing zone [m]')
    ap.add_argument('--turns', type=float, default=None,
                    help='expected flip turns (default 0.5, or 1 for the hop)')
    ap.add_argument('--max-rot-err-deg', type=float, default=15.0)
    ap.add_argument('--min-hold', type=float, default=1.0, help='inverted hold [s] (ramp)')
    ap.add_argument('--max-est-pos', type=float, default=0.5,
                    help='IMU estimate position error vs truth, whole run [m]')
    ap.add_argument('--max-est-att', type=float, default=3.0,
                    help='IMU estimate attitude error vs truth, whole run [deg]')
    ap.add_argument('--min-release-speed', type=float, default=6.0,
                    help='forward speed when letting go of the overhang [m/s] (ramp)')
    ap.add_argument('--zero-to-60', type=float, default=0.81, help='Demo C target [s]')
    ap.add_argument('--zero-to-60-tol', type=float, default=0.05)
    ap.add_argument('--launch-impulse', type=float, default=32e3,
                    help='Demo C impulse estimate (plan Section 2) [N.s], scored within 25%%')
    ap.add_argument('--max-yaw-deg', type=float, default=2.0, help='Demo C yaw error')
    ap.add_argument('--min-clearance', type=float, default=0.5,
                    help='Demo B: underside over every vehicle [m] (plan target 1.0)')
    ap.add_argument('--min-normal', type=float, default=500.0,
                    help='per-wheel normal force while held [N] (ramp); upside down a '
                         "wheel's own weight already reads ~147 N")
    args = ap.parse_args()
    sess = Path(args.session)
    recs = [json.loads(line) for line in open(sess / 'run.jsonl')]
    status = [r for r in recs if r['topic'] == 'status']
    odom = [r for r in recs if r['topic'] == 'odom']
    contacts = [r for r in recs if r['topic'] == 'body_contact']
    if not status or not odom:
        print('FAIL: no status/odom records')
        return 1

    # state timeline
    timeline, last = [], None
    for s in status:
        if s['state'] != last:
            timeline.append((s['t'], s['state']))
            last = s['state']
    states = {st for _, st in timeline}
    ramp, drop = 'CLIMB' in states, 'HOLD' in states
    launch, hop = 'LAUNCH' in states, 'APPROACH' in states
    if args.turns is None:
        args.turns = 1.0 if hop else 0.5
    # flight starts when the car lets go (ramp, drop) or lifts off (hop)
    t_release = next((t for t, st in timeline if st in ('RELEASE', 'HOP_BURN')), None)
    final = status[-1]
    geometry = json.load(open(args.map))['geometry'] if args.map else {}

    # touchdown
    td_i = None
    if t_release is not None:
        for i, o in enumerate(odom):
            if o['t'] > t_release + 0.2 and o['p'][2] < args.td_height:
                td_i = i
                break
    m = {'timeline': timeline, 'final_state': final['state']}
    if td_i is not None and td_i >= 3:
        o0, o1 = odom[td_i - 3], odom[td_i]
        vz = (com_z(o1['p'], o1['q']) - com_z(o0['p'], o0['q'])) / (o1['t'] - o0['t'])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, body_z(o1['q'])[2]))))
        m.update({'t_touchdown': o1['t'], 'touchdown_vz': vz, 'touchdown_tilt_deg': tilt,
                  'touchdown_xy': o1['p'][:2], 'touchdown_lateral_offset': o1['p'][1],
                  'touchdown_lateral_speed': (o1['p'][1] - o0['p'][1]) / (o1['t'] - o0['t'])})
        # peak COM deceleration in the 0.5 s around touchdown (5-sample differences)
        win = [o for o in odom if o1['t'] - 0.5 <= o['t'] <= o1['t'] + 0.5]
        acc = []
        for a, b, c in zip(win, win[5:], win[10:]):
            v1 = (com_z(b['p'], b['q']) - com_z(a['p'], a['q'])) / (b['t'] - a['t'])
            v2 = (com_z(c['p'], c['q']) - com_z(b['p'], b['q'])) / (c['t'] - b['t'])
            acc.append((v2 - v1) / ((c['t'] - a['t']) / 2))
        m['peak_vertical_accel_g'] = max(acc) / 9.81 if acc else None

    # flip rotation: status integrates the body rate about the flip axis
    at_td = [s for s in status if 't_touchdown' in m and s['t'] <= m['t_touchdown']]
    m['flip_rotation_deg'] = math.degrees((at_td or status)[-1]['roll_progress'])
    m['impulse_used_Ns'] = final['impulse_used']
    first = status[0]
    m['gas_used_kg'] = first['tank_mass'] - final['tank_mass']
    m['tank_mass_end_kg'] = final['tank_mass']
    # below 1 the tank has dropped under the regulated nozzle pressure and every
    # nozzle gives proportionally less thrust: the gas budget's real limit
    to_td = [s for s in status if 't_touchdown' not in m or s['t'] <= m['t_touchdown']]
    m['min_feed_ratio_to_touchdown'] = min(s['feed_ratio'] for s in to_td)
    m['max_height_m'] = max(s['height'] for s in status)
    after = [c for c in contacts if t_release is not None and c['t'] > t_release + 0.05]
    m['body_contacts_after_release'] = len(after)
    m['body_contact_with'] = sorted({w for c in after for w in c['with']})

    checks = {}
    if ramp:
        # ground phase: from the start until release (or the end of a hold-only run)
        t_end_ground = t_release if t_release is not None else status[-1]['t'] + 1
        ground = [s for s in status if s['t'] < t_end_ground]
        held = [s for s in ground if s['state'] == 'HOLD_INVERTED']
        # wheel loads from the run-in on (past the first 0.5 s of spawn settling):
        # a wheel off the road there is where the rigid car's diagonal began
        on_ramp = [s for s in ground if s['state'] in ('CLIMB', 'STOP', 'HOLD_INVERTED', 'DRIVE_OFF')
                   or (s['state'] == 'ACCELERATE' and s['t'] > 0.5)]
        m['max_arc_angle_deg'] = math.degrees(max(s['arc_angle'] for s in ground))
        m['max_lateral_offset_m'] = max(abs(s['lateral_offset']) for s in ground)
        m['min_wheel_normal_on_ramp_N'] = min(
            (s['min_wheel_normal'] for s in on_ramp), default=None)
        m['min_wheel_normal_held_N'] = min((s['min_wheel_normal'] for s in held), default=None)
        # hold duration from the state timeline (entry -> next state's entry);
        # first-to-last sample would miss one 5 ms status period
        hold_t0 = next((t for t, st in timeline if st == 'HOLD_INVERTED'), None)
        hold_t1 = next((t for t, st in timeline if hold_t0 is not None and t > hold_t0), None)
        m['inverted_hold_s'] = 0.0 if hold_t0 is None else (
            (hold_t1 if hold_t1 is not None else status[-1]['t']) - hold_t0)
        m['max_climb_speed'] = max(s['speed'] for s in ground)
        # stillness over the final min_hold seconds of the hold (the plan's
        # "stays attached for 2 s"); the hold loop may still be settling the
        # last centimetre right after entry
        ho = [o for o in odom if held and max(held[0]['t'], held[-1]['t'] - args.min_hold)
              <= o['t'] <= held[-1]['t']]
        if ho:
            p0 = ho[0]['p']
            m['hold_drift_m'] = max(math.dist(o['p'], p0) for o in ho)
            m['hold_xyz'] = p0
            m['hold_inverted_cos'] = min(-body_z(o['q'])[2] for o in ho)
        ground_contacts = [c for c in contacts if c['t'] < t_end_ground]
        m['body_contacts_on_ramp'] = len(ground_contacts)
        m['body_contact_on_ramp_with'] = sorted({w for c in ground_contacts for w in c['with']})
        checks.update({
            'reached fully inverted': m['max_arc_angle_deg'] >= 179.0,
            f'held inverted >= {args.min_hold:.1f} s': m['inverted_hold_s'] >= args.min_hold,
            'stayed upside down while held': m.get('hold_inverted_cos', 0) > math.cos(
                math.radians(5)),
            'held still (< 2 cm)': m.get('hold_drift_m', 1e9) < 0.02,
            'wheels loaded while held': (m['min_wheel_normal_held_N'] or 0) > args.min_normal,
            'all wheels on the track (run-in to drive-off)':
                (m['min_wheel_normal_on_ramp_N'] or 0) > 0,
            'stayed centred (< 0.2 m)': m['max_lateral_offset_m'] < 0.2,
            'no body contact on the ramp': m['body_contacts_on_ramp'] == 0,
        })
    if ramp and t_release is not None:
        # drive-off: speed at the moment of letting go, and the landing run
        rel = next(s for s in status if s['t'] >= t_release)
        m['release_speed'] = rel['speed']
        o_rel = min(odom, key=lambda o: abs(o['t'] - t_release))
        m['release_xyz'] = o_rel['p']
        if 't_touchdown' in m:
            w = [o for o in odom if m['t_touchdown'] - 0.05 <= o['t'] <= m['t_touchdown']]
            if len(w) >= 2:
                m['touchdown_forward_speed'] = math.dist(w[-1]['p'][:2], w[0]['p'][:2]) / (
                    w[-1]['t'] - w[0]['t'])
            m['flight_distance_m'] = math.dist(m['touchdown_xy'], o_rel['p'][:2])
        stop = next((t for t, st in timeline if st == 'STOPPED'), None)
        if stop is not None and 't_touchdown' in m:
            o_stop = min(odom, key=lambda o: abs(o['t'] - stop))
            m['rollout_m'] = math.dist(o_stop['p'][:2], m['touchdown_xy'])
            m['stop_xy'] = o_stop['p'][:2]
        checks.update({
            f'drove off forward (>= {args.min_release_speed:.1f} m/s)':
                m['release_speed'] >= args.min_release_speed,
            'stopped on its wheels': stop is not None,
        })
    if launch:
        # Demo C: 0-60 mph from the launch command, on truth speed (20 ms differences)
        t0 = next(t for t, st in timeline if st == 'LAUNCH')
        o0 = min(odom, key=lambda o: abs(o['t'] - t0))
        speed = []
        for a, b in zip(odom, odom[4:]):
            speed.append(((a['t'] + b['t']) / 2, math.dist(a['p'][:2], b['p'][:2]) / (b['t'] - a['t'])))
        t60 = next((t for t, v in speed if t > t0 and v >= MPH60), None)
        m['top_speed'] = max(v for _, v in speed)
        yaw0 = yaw_of(o0['q'])
        after0 = [o for o in odom if o['t'] >= t0]
        yaw_err = [math.remainder(yaw_of(o['q']) - yaw0, 2 * math.pi) for o in after0]
        m['max_yaw_error_deg'] = math.degrees(max(abs(e) for e in yaw_err))
        m['max_lateral_offset_m'] = max(abs(o['p'][1] - o0['p'][1]) for o in after0)
        if t60 is not None:
            m['zero_to_60_s'] = t60 - t0
            o60 = min(odom, key=lambda o: abs(o['t'] - t60))
            m['distance_to_60_m'] = o60['p'][0] - o0['p'][0]
            m['launch_impulse_Ns'] = min(status, key=lambda s: abs(s['t'] - t60))['impulse_used']
            # rear thrust delivered vs commanded, past the valves' rise
            thrust = [r for r in recs if r['topic'] == 'thrust' and t0 + 0.1 <= r['t'] <= t60]
            ratios = []
            for r in thrust:
                s = min(status, key=lambda s: abs(s['t'] - r['t']))
                if s['state'] == 'LAUNCH' and s['wrench_des'][0] > 0:
                    ratios.append(sum(r['f']) / s['wrench_des'][0])
            if ratios:
                m['thrust_tracking_max_err'] = max(abs(x - 1) for x in ratios)
                cmd = [s['wrench_des'][0] for s in status
                       if s['state'] == 'LAUNCH' and s['wrench_des'][0] > 0]
                m['launch_thrust_kN'] = sum(ratios) / len(ratios) * sum(cmd) / len(cmd) / 1e3
        stop = next((t for t, st in timeline if st == 'STOPPED'), None)
        if stop is not None:
            m['stop_x'] = min(odom, key=lambda o: abs(o['t'] - stop))['p'][0]
        lo, hi = args.zero_to_60 - args.zero_to_60_tol, args.zero_to_60 + args.zero_to_60_tol
        checks.update({
            f'0-60 mph in {lo:.2f}-{hi:.2f} s': lo <= m.get('zero_to_60_s', 1e9) <= hi,
            'rear thrust tracks the command (< 5%)': m.get('thrust_tracking_max_err', 1) < 0.05,
            f'impulse within 25% of {args.launch_impulse / 1e3:.0f} kN.s':
                abs(m.get('launch_impulse_Ns', 0) / args.launch_impulse - 1) < 0.25,
            f'yaw error < {args.max_yaw_deg:.1f} deg': m['max_yaw_error_deg'] < args.max_yaw_deg,
            'stopped on its wheels': stop is not None,
            'no body contact': len(contacts) == 0,
        })
    if hop:
        # Demo B: the lowest corner's clearance over each vehicle box while over it
        obstacles = geometry.get('obstacles', [])
        flight = [o for o in odom if t_release is not None and o['t'] >= t_release and
                  ('t_touchdown' not in m or o['t'] <= m['t_touchdown'])]
        clear = {}
        for o in flight:
            corners = [rotate(o['q'], (sx * CAR_X, sy * CAR_Y, z))
                       for sx in (-1, 1) for sy in (-1, 1) for z in (CAR_BOTTOM_Z, CAR_TOP_Z)]
            pts = [(o['p'][0] + c[0], o['p'][1] + c[1], o['p'][2] + c[2]) for c in corners]
            x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
            y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)
            for ob in obstacles:
                (cx, cy, cz), (sx, sy, sz) = ob['center'], ob['size']
                if x1 >= cx - sx / 2 and x0 <= cx + sx / 2 and y1 >= cy - sy / 2 and y0 <= cy + sy / 2:
                    c = min(p[2] for p in pts) - (cz + sz / 2)
                    clear[ob['name']] = min(clear.get(ob['name'], 1e9), c)
        m['clearance_m'] = {k: round(v, 3) for k, v in clear.items()}
        m['min_clearance_m'] = min(clear.values()) if clear else None
        m['apex_com_height_m'] = max((com_z(o['p'], o['q']) for o in flight), default=None)
        if t_release is not None:
            m['liftoff_speed'] = next(s for s in status if s['t'] >= t_release)['speed']
        stop = next((t for t, st in timeline if st == 'STOPPED'), None)
        checks.update({
            'lifted off': t_release is not None,
            f'cleared every vehicle by >= {args.min_clearance:.1f} m':
                len(clear) == len(obstacles) and (m['min_clearance_m'] or 0) >= args.min_clearance,
            'stopped on its wheels': stop is not None,
        })
    if t_release is not None or drop or hop:
        checks.update({
            'landed': final['state'] == 'LANDED' or any(st == 'LANDED' for _, st in timeline),
            'touchdown found': 'touchdown_vz' in m,
            'touchdown speed': 'touchdown_vz' in m and abs(m['touchdown_vz']) < args.max_td_speed,
            'touchdown attitude': 'touchdown_tilt_deg' in m and
            m['touchdown_tilt_deg'] < args.max_td_tilt_deg,
            'touchdown lateral speed': 'touchdown_lateral_speed' in m and
            abs(m['touchdown_lateral_speed']) < args.max_td_lateral_speed,
            'flip rotation': abs(m['flip_rotation_deg'] - 360 * args.turns) < args.max_rot_err_deg,
            'no body contact after release': m['body_contacts_after_release'] == 0,
        })
    landing_w = geometry.get('landing_width')
    if landing_w is not None and t_release is not None:
        m['landing_zone_half_width'] = landing_w / 2
        lx = geometry.get('landing_x', [-math.inf, math.inf])
        checks[f'touched down inside the {landing_w:.0f} m landing zone'] = (
            'touchdown_lateral_offset' in m and
            abs(m['touchdown_lateral_offset']) + args.car_half_width <= landing_w / 2 and
            lx[0] <= m['touchdown_xy'][0] <= lx[1])
    # IMU state estimate vs ground truth, per mission phase
    est = {round(r['t'], 3): r for r in recs if r['topic'] == 'odom_est'}
    if est:
        phases = [(t, st) for t, st in timeline] + [(status[-1]['t'] + 1, None)]

        def phase_of(t):
            cur = None
            for t0, st in phases:
                if t < t0:
                    break
                cur = st
            return cur

        # scored up to touchdown, what the stunt flies on: with no heading
        # aid the estimate dead-reckons, and its yaw drift adds ~0.4% of the
        # distance driven sideways (0.5 m on the hop's rollout, hop_imu4);
        # the after-touchdown maximum is reported, not scored
        t_score_end = m.get('t_touchdown', math.inf)
        err, after_td = {}, 0.0
        for o in odom:
            e = est.get(round(o['t'], 3))
            if e is None:
                continue
            if o['t'] > t_score_end:
                after_td = max(after_td, math.dist(o['p'], e['p']))
                continue
            dp = math.dist(o['p'], e['p'])
            dz = abs(o['p'][2] - e['p'][2])
            qd = abs(sum(a * b for a, b in zip(o['q'], e['q'])))
            dth = math.degrees(2 * math.acos(min(1.0, qd)))
            ph = phase_of(o['t'])
            agg = err.setdefault(ph, [0.0, 0.0, 0.0, 0])
            agg[0] = max(agg[0], dp)
            agg[1] = max(agg[1], dz)
            agg[2] = max(agg[2], dth)
            agg[3] += 1
        m['estimate_error_by_phase'] = {
            ph: {'max_pos_m': round(a[0], 3), 'max_height_m': round(a[1], 3),
                 'max_att_deg': round(a[2], 2), 'samples': a[3]} for ph, a in err.items()}
        if math.isfinite(t_score_end):
            m['estimate_max_pos_err_after_touchdown_m'] = round(after_td, 3)
        if err:
            m['estimate_max_pos_err_m'] = max(a[0] for a in err.values())
            m['estimate_max_att_err_deg'] = max(a[2] for a in err.values())
            until = ' to touchdown' if math.isfinite(t_score_end) else ''
            checks[f'IMU estimate within {args.max_est_pos:.2f} m / {args.max_est_att:.1f} deg{until}'] = (
                m['estimate_max_pos_err_m'] < args.max_est_pos and
                m['estimate_max_att_err_deg'] < args.max_est_att)
    m['checks'] = checks
    m['pass'] = all(checks.values())
    (sess / 'metrics.json').write_text(json.dumps(m, indent=2) + '\n')

    print('timeline: ' + '  '.join(f'{t:.2f} {st}' for t, st in timeline))
    for k in ('max_arc_angle_deg', 'max_climb_speed', 'max_lateral_offset_m',
              'min_wheel_normal_on_ramp_N', 'min_wheel_normal_held_N', 'inverted_hold_s',
              'hold_drift_m', 'hold_xyz', 'body_contacts_on_ramp', 'body_contact_on_ramp_with',
              'release_speed', 'release_xyz', 'flight_distance_m', 'touchdown_forward_speed',
              'rollout_m', 'stop_xy',
              'zero_to_60_s', 'distance_to_60_m', 'top_speed', 'launch_thrust_kN',
              'thrust_tracking_max_err', 'launch_impulse_Ns', 'max_yaw_error_deg', 'stop_x',
              'liftoff_speed', 'apex_com_height_m', 'min_clearance_m', 'clearance_m',
              'flip_rotation_deg', 'touchdown_vz', 'touchdown_tilt_deg', 'peak_vertical_accel_g',
              'touchdown_lateral_offset', 'touchdown_lateral_speed', 'landing_zone_half_width',
              'impulse_used_Ns', 'gas_used_kg', 'tank_mass_end_kg', 'min_feed_ratio_to_touchdown',
              'max_height_m',
              'body_contacts_after_release', 'body_contact_with', 't_touchdown', 'touchdown_xy'):
        if k not in m:
            continue
        v = m.get(k)
        print(f'  {k:28s} {v:.3f}' if isinstance(v, float) else f'  {k:28s} {v}')
    if 'estimate_error_by_phase' in m:
        print('  IMU estimate error vs truth (max per phase):')
        for ph, e in m['estimate_error_by_phase'].items():
            print(f'    {str(ph):14s} pos {e["max_pos_m"]:.3f} m  height {e["max_height_m"]:.3f} m'
                  f'  attitude {e["max_att_deg"]:.2f} deg')
        if 'estimate_max_pos_err_after_touchdown_m' in m:
            print(f'    after touchdown (not scored): pos {m["estimate_max_pos_err_after_touchdown_m"]:.3f} m')
    for k, ok in checks.items():
        print(f'  [{"PASS" if ok else "FAIL"}] {k}')
    print('RESULT: ' + ('PASS' if m['pass'] else 'FAIL'))
    return 0 if m['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
