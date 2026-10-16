#!/usr/bin/env python3
"""analyze_stunt_run.py - metrics + pass/fail for one recorded stunt run.

  python3 tools/avkit/analyze_stunt_run.py data/stunt_runs/<session>

Reads <session>/run.jsonl (record_stunt_run.py), writes <session>/metrics.json
and prints a report. Metrics (plan section 9): flip rotation, touchdown
vertical speed and attitude, peak landing deceleration, body (non-wheel)
contacts after release, impulse and gas used, state timeline. Ramp runs
(timeline starts with ACCELERATE) are also scored on the ground phase: arc
progress, inverted hold time / stillness / wheel loads, lateral offset and
body contacts on the ramp; flight checks apply only when the car released.

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


def body_z(q):
    x, y, z, w = q
    return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))


def com_z(p, q):
    return p[2] + COM_Z * body_z(q)[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session')
    ap.add_argument('--td-height', type=float, default=0.03)
    ap.add_argument('--max-td-speed', type=float, default=3.0)
    ap.add_argument('--max-td-tilt-deg', type=float, default=10.0)
    ap.add_argument('--turns', type=float, default=0.5, help='expected flip turns')
    ap.add_argument('--max-rot-err-deg', type=float, default=15.0)
    ap.add_argument('--min-hold', type=float, default=1.0, help='inverted hold [s] (ramp)')
    ap.add_argument('--max-est-pos', type=float, default=0.5,
                    help='IMU estimate position error vs truth, whole run [m]')
    ap.add_argument('--max-est-att', type=float, default=3.0,
                    help='IMU estimate attitude error vs truth, whole run [deg]')
    ap.add_argument('--min-release-speed', type=float, default=6.0,
                    help='forward speed when letting go of the overhang [m/s] (ramp)')
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
    t_release = next((t for t, st in timeline if st == 'RELEASE'), None)
    final = status[-1]

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
                  'touchdown_xy': o1['p'][:2]})
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
    m['max_height_m'] = max(s['height'] for s in status)
    after = [c for c in contacts if t_release is not None and c['t'] > t_release + 0.05]
    m['body_contacts_after_release'] = len(after)
    m['body_contact_with'] = sorted({w for c in after for w in c['with']})

    checks = {}
    ramp = any(st == 'ACCELERATE' for _, st in timeline)
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
    if t_release is not None or not ramp:
        checks.update({
            'landed': final['state'] == 'LANDED' or any(st == 'LANDED' for _, st in timeline),
            'touchdown found': 'touchdown_vz' in m,
            'touchdown speed': 'touchdown_vz' in m and abs(m['touchdown_vz']) < args.max_td_speed,
            'touchdown attitude': 'touchdown_tilt_deg' in m and
            m['touchdown_tilt_deg'] < args.max_td_tilt_deg,
            'flip rotation': abs(m['flip_rotation_deg'] - 360 * args.turns) < args.max_rot_err_deg,
            'no body contact after release': m['body_contacts_after_release'] == 0,
        })
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

        err = {}
        for o in odom:
            e = est.get(round(o['t'], 3))
            if e is None:
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
        if err:
            m['estimate_max_pos_err_m'] = max(a[0] for a in err.values())
            m['estimate_max_att_err_deg'] = max(a[2] for a in err.values())
            checks[f'IMU estimate within {args.max_est_pos:.2f} m / {args.max_est_att:.1f} deg'] = (
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
              'flip_rotation_deg', 'touchdown_vz', 'touchdown_tilt_deg', 'peak_vertical_accel_g',
              'impulse_used_Ns', 'gas_used_kg', 'tank_mass_end_kg', 'max_height_m',
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
    for k, ok in checks.items():
        print(f'  [{"PASS" if ok else "FAIL"}] {k}')
    print('RESULT: ' + ('PASS' if m['pass'] else 'FAIL'))
    return 0 if m['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
