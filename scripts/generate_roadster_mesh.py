#!/usr/bin/env python3
"""generate_roadster_mesh.py - visual meshes of the roadster (body + wheel).

Visual only: the car's collision box, inertia, frames and wheel geometry stay
in roadster.urdf.xacro; these meshes replace its box/cylinder visuals.

Shape: a faceted low-poly replica of the second-generation Roadster as Tesla
showed it at the 2026-10-15 unveil (press photos: stark white angular wedge,
black lower body and sills, bronze-tinted glass from the cowl to a fastback
between white rear buttresses, full-width light bars - white front, red rear -,
four brass nozzles in a black rear fascia, white aero wheel discs). No
dimensions are published, so the side profile was measured off the side-on
press photo scaled by its wheelbase to this model's 2.7 m; the result is
close to the photo's own proportions (4.3 m x 1.93 m x 1.08 m). The body is
lofted through cross-sections, which suits the car's flat facets.

Fitted to the simulated car, not the other way round:
- wheel arches clear the wheels at full suspension compression (wheel top
  0.66 m + 0.12 m travel) and full steering lock;
- the rear fascia sits just inside the rear_main_* nozzle exits of
  config/nozzles.yaml (read here), whose brass rings mark the exact points
  the thrust and the plume emitters come from; the down_* nozzles get rings
  on the underside, where they show when the car hangs inverted;
- the underside stays 0.14 m above the ground (the suspension compresses
  ~0.08 m at the ramp bottom).

Frame: the `chassis` link (x forward, y left, z up, origin wheel_radius =
0.33 m above ground at rest), so the URDF attaches the mesh with no offset.
Coordinates below are written in metres above the GROUND and shifted on output.

Outputs (COLLADA, one <triangles> per material, flat-shaded):
  src/roadster_description/meshes/roadster_body.dae
  src/roadster_description/meshes/roadster_wheel.dae  (axis along y, centred)
Usage:
  python3 scripts/generate_roadster_mesh.py [--preview DIR]
--preview also writes orthographic PNG views (side, top, front, rear,
three-quarter) rendered with Pillow, for shaping without launching Gazebo.
"""
import argparse
import math
from pathlib import Path

import numpy as np
import yaml

WS = Path(__file__).resolve().parents[1]
PKG = WS / 'src' / 'roadster_description'
MESH_DIR = PKG / 'meshes'

# must match roadster.urdf.xacro
WHEELBASE, TRACK, WHEEL_R, WHEEL_W = 2.7, 1.6, 0.33, 0.25

# name: (diffuse rgb, specular, emission rgb or None)
MATERIALS = {
    'paint': ((0.86, 0.87, 0.88), 0.5, None),
    'trim': ((0.035, 0.035, 0.04), 0.2, None),       # lower body, sills, fascia
    'glass': ((0.11, 0.07, 0.035), 0.6, None),       # bronze tint
    'well': ((0.02, 0.02, 0.02), 0.0, None),         # wheel wells, nozzle bores
    'brass': ((0.70, 0.50, 0.22), 0.8, None),
    'fan': ((0.12, 0.12, 0.13), 0.3, None),
    'tail_light': ((0.9, 0.02, 0.02), 0.3, (1.0, 0.05, 0.03)),
    'head_light': ((0.95, 0.97, 1.0), 0.3, (1.0, 1.0, 1.0)),
    'tyre': ((0.04, 0.04, 0.045), 0.1, None),
    'disc': ((0.90, 0.91, 0.92), 0.5, None),
    'rim': ((0.10, 0.10, 0.11), 0.4, None),
}

# Half cross-sections, front to rear. Each row: x, then (y, z) of
#   P1 floor edge, P2 sill top (black below), P3 side crease, P4 shoulder,
#   P5 greenhouse base, P6 roof edge, and z of the top centreline P7.
# P0 is the floor centre (0, z of P1). Values in metres above the ground.
SECTIONS = [
    # x      P1          P2          P3           P4          P5          P6          P7
    (2.15, .52, .16, .56, .26, .58, .40, .56, .50, .40, .505, .20, .50, .49),
    (1.98, .82, .14, .90, .27, .93, .42, .90, .60, .64, .62, .30, .60, .58),
    (1.75, .88, .14, .95, .28, .96, .45, .93, .69, .68, .71, .32, .67, .65),
    (1.35, .88, .14, .95, .29, .965, .47, .94, .75, .70, .77, .34, .73, .72),
    (0.95, .88, .14, .95, .30, .965, .48, .94, .77, .76, .82, .48, .90, .88),
    (0.40, .88, .14, .95, .30, .965, .47, .94, .74, .82, .82, .58, 1.02, 1.03),
    (-0.30, .88, .14, .95, .30, .965, .47, .95, .75, .84, .82, .60, 1.06, 1.08),
    (-0.95, .88, .14, .95, .31, .965, .50, .96, .80, .80, .86, .50, 1.00, 1.01),
    (-1.35, .88, .14, .95, .33, .965, .53, .96, .84, .72, .88, .42, .94, .94),
    (-1.75, .88, .15, .95, .35, .96, .56, .95, .86, .66, .88, .36, .88, .88),
    (-2.05, .86, .18, .93, .38, .95, .58, .94, .84, .70, .85, .34, .84, .84),
    (-2.20, .80, .22, .86, .38, .90, .58, .88, .78, .66, .79, .32, .79, .79),
]

# Wheel arches (trapezoid, like the car's angular arches): half-length at the
# sill, half-length of the flat top, top height, height at the opening edge,
# and the inner wall's y (inside the tyre's inner face, 0.675 m).
ARCH_HALF, ARCH_TOP_HALF, ARCH_TOP_Z, ARCH_EDGE_Z, ARCH_INNER_Y = 0.40, 0.18, 0.77, 0.56, 0.62
EPS = 0.004

# Material of each profile edge P_k -> P_k+1 (k = 0..6) as a function of x
GLASS_SIDE = (-1.0, 1.40)    # side windows and A-pillars (edge P5-P6)
GLASS_TOP = (-1.75, 1.40)    # windscreen, roof and fastback glass (edge P6-P7)
REAR_TRIM_X = -1.75          # behind this the lower body is black up to P3


def edge_material(k, x0, x1, arch):
    xm = 0.5 * (x0 + x1)
    if k == 0:
        return 'well' if arch else 'trim'
    if k == 1:
        return 'well' if arch else 'trim'
    if k == 2:
        return 'trim' if xm < REAR_TRIM_X else 'paint'
    if k == 5 and GLASS_SIDE[0] < xm < GLASS_SIDE[1]:
        return 'glass'
    if k == 6 and GLASS_TOP[0] < xm < GLASS_TOP[1]:
        return 'glass'
    return 'paint'


class Mesh:
    """Triangles grouped by material; flat-shaded on export."""

    def __init__(self):
        self.tris = {}

    def tri(self, mat, a, b, c):
        self.tris.setdefault(mat, []).append((np.asarray(a, float), np.asarray(b, float),
                                              np.asarray(c, float)))

    def quad(self, mat, a, b, c, d):
        """a-b-c-d counter-clockwise seen from the outside."""
        self.tri(mat, a, b, c)
        self.tri(mat, a, c, d)

    def poly(self, mat, pts, normal):
        """Convex planar polygon, fan-triangulated, wound to face `normal`."""
        pts = [np.asarray(p, float) for p in pts]
        c = sum(pts) / len(pts)
        for i in range(len(pts)):
            a, b = pts[i], pts[(i + 1) % len(pts)]
            if np.dot(np.cross(a - c, b - c), normal) >= 0:
                self.tri(mat, c, a, b)
            else:
                self.tri(mat, c, b, a)

    def ring(self, mat, centre, axis, r_in, r_out, n=24):
        """Flat annulus (or disc for r_in = 0) facing `axis`."""
        centre, axis = np.asarray(centre, float), np.asarray(axis, float)
        u, v = basis(axis)
        for i in range(n):
            t0, t1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            o0 = centre + r_out * (math.cos(t0) * u + math.sin(t0) * v)
            o1 = centre + r_out * (math.cos(t1) * u + math.sin(t1) * v)
            if r_in <= 0:
                self.tri(mat, centre, o0, o1)
            else:
                i0 = centre + r_in * (math.cos(t0) * u + math.sin(t0) * v)
                i1 = centre + r_in * (math.cos(t1) * u + math.sin(t1) * v)
                self.quad(mat, i0, o0, o1, i1)

    def tube(self, mat, centre, axis, r, length, n=24, inward=False):
        """Open cylinder from centre to centre + axis * length."""
        centre, axis = np.asarray(centre, float), np.asarray(axis, float)
        u, v = basis(axis)
        for i in range(n):
            t0, t1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            p0 = centre + r * (math.cos(t0) * u + math.sin(t0) * v)
            p1 = centre + r * (math.cos(t1) * u + math.sin(t1) * v)
            q0, q1 = p0 + axis * length, p1 + axis * length
            if inward:
                self.quad(mat, p0, q0, q1, p1)
            else:
                self.quad(mat, p0, p1, q1, q0)

    def shifted(self, dz):
        out = Mesh()
        off = np.array([0.0, 0.0, dz])
        out.tris = {m: [(a + off, b + off, c + off) for a, b, c in t] for m, t in self.tris.items()}
        return out


def basis(axis):
    axis = axis / np.linalg.norm(axis)
    helper = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(helper, axis)
    u /= np.linalg.norm(u)
    # (u, v, axis) right-handed, so a counter-clockwise (u -> v) loop faces +axis
    return u, np.cross(axis, u)


# ----------------------------------------------------------------- body

def section_at(x):
    """Half-profile [P0..P7] as (y, z) pairs at x, interpolated between rows."""
    xs = [s[0] for s in SECTIONS]
    if x >= xs[0]:
        row = np.array(SECTIONS[0][1:])
    elif x <= xs[-1]:
        row = np.array(SECTIONS[-1][1:])
    else:
        i = next(i for i in range(len(xs) - 1) if xs[i] >= x >= xs[i + 1])
        t = (xs[i] - x) / (xs[i] - xs[i + 1])
        row = (1 - t) * np.array(SECTIONS[i][1:]) + t * np.array(SECTIONS[i + 1][1:])
    p = [(0.0, row[1])] + [(row[2 * k], row[2 * k + 1]) for k in range(6)] + [(0.0, row[12])]
    return p


def arch_z(dx):
    """Arch height above the ground at distance dx from the wheel centre."""
    dx = abs(dx)
    if dx <= ARCH_TOP_HALF:
        return ARCH_TOP_Z
    t = (dx - ARCH_TOP_HALF) / (ARCH_HALF - ARCH_TOP_HALF)
    return ARCH_TOP_Z + t * (ARCH_EDGE_Z - ARCH_TOP_Z)


def wheel_centres_x():
    return (WHEELBASE / 2, -WHEELBASE / 2)


def in_arch(x):
    for c in wheel_centres_x():
        if abs(x - c) < ARCH_HALF:
            return x - c
    return None


def archify(p, dx):
    """Cut the wheel opening out of a half-profile: the floor edge moves in to
    the inner wall, the sill and crease rise to the arch."""
    a = arch_z(dx)
    p = list(p)
    p[1] = (ARCH_INNER_Y, a)
    p[2] = (p[2][0], a)
    p[3] = (p[3][0], max(p[3][1], a + 0.03))
    return p


def body_stations():
    xs = {s[0] for s in SECTIONS}
    for c in wheel_centres_x():
        for d in (ARCH_HALF + EPS, ARCH_HALF - EPS, ARCH_TOP_HALF, 0.0):
            xs.update((c + d, c - d))
    out = []
    for x in sorted(xs, reverse=True):
        p = section_at(x)
        dx = in_arch(x)
        out.append((x, archify(p, dx) if dx is not None else p, dx is not None))
    return out


def side_pt(x, yz, sign):
    return (x, sign * yz[0], yz[1])


def build_body(nozzles):
    m = Mesh()
    st = body_stations()
    for (x0, p0, a0), (x1, p1, a1) in zip(st, st[1:]):
        for k in range(7):
            mat = edge_material(k, x0, x1, a0 and a1)
            for s in (1, -1):
                A, B = side_pt(x0, p0[k], s), side_pt(x0, p0[k + 1], s)
                C, D = side_pt(x1, p1[k + 1], s), side_pt(x1, p1[k], s)
                # profile runs floor -> roof on the +y side; (A, D, C, B) faces out
                if s > 0:
                    m.quad(mat, A, D, C, B)
                else:
                    m.quad(mat, A, B, C, D)
    # end caps: nose painted, rear fascia black
    for (x, p, _), normal, mat in ((st[0], (1, 0, 0), 'paint'), (st[-1], (-1, 0, 0), 'trim')):
        ring = [side_pt(x, q, 1) for q in p] + [side_pt(x, q, -1) for q in reversed(p[1:-1])]
        m.poly(mat, ring, np.array(normal, float))

    x_nose = SECTIONS[0][0]
    x_tail = SECTIONS[-1][0]
    # full-width light bars, standing just proud of the caps
    m.poly('head_light', [(x_nose + 0.003, y, z) for y, z in
                          ((-0.50, 0.43), (0.50, 0.43), (0.50, 0.455), (-0.50, 0.455))],
           np.array([1.0, 0, 0]))
    m.poly('tail_light', [(x_tail - 0.003, y, z) for y, z in
                          ((-0.84, 0.735), (0.84, 0.735), (0.84, 0.765), (-0.84, 0.765))],
           np.array([-1.0, 0, 0]))
    # brass nozzle rings at the configured nozzle exits (chassis frame + 0.33)
    for name, n in sorted(nozzles.items()):
        x, y, z = n['xyz']
        z += WHEEL_R
        if name.startswith('rear_main'):
            back = np.array([-1.0, 0, 0])
            m.tube('brass', (x_tail, y, z), back, 0.065, x - x_tail)
            m.tube('well', (x_tail, y, z), back, 0.045, x - x_tail, inward=True)
            m.ring('brass', (x, y, z), back, 0.045, 0.065)
            m.ring('well', (x_tail - 0.002, y, z), back, 0.0, 0.045)
        elif name.startswith('down'):
            zb = section_at(x)[0][1] - 0.002
            m.ring('brass', (x, y, zb), np.array([0, 0, -1.0]), 0.05, 0.075)
            m.ring('well', (x, y, zb), np.array([0, 0, -1.0]), 0.0, 0.05)
    # suction fan intakes on the floor (four, as reported; positions ours)
    for fx in (-0.25, -0.75):
        for fy in (0.32, -0.32):
            zb = section_at(fx)[0][1] - 0.002
            m.ring('rim', (fx, fy, zb), np.array([0, 0, -1.0]), 0.19, 0.22, n=32)
            m.ring('fan', (fx, fy, zb), np.array([0, 0, -1.0]), 0.0, 0.19, n=32)
    return m.shifted(-WHEEL_R)


# ----------------------------------------------------------------- wheel

def build_wheel(n=40):
    """Tyre with chamfered shoulders, white aero disc on both faces (the inner
    one is hidden), dark rim ring and hub. Axis along y, centre at origin."""
    m = Mesh()
    h = WHEEL_W / 2
    # tyre cross-section (r, y), outer surface, traversed so it faces outward
    prof = [(0.255, h - 0.01), (0.30, h), (WHEEL_R, h - 0.03),
            (WHEEL_R, -h + 0.03), (0.30, -h), (0.255, -h + 0.01)]
    for i in range(n):
        t0, t1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        for (r0, y0), (r1, y1) in zip(prof, prof[1:]):
            a = (r0 * math.cos(t0), y0, r0 * math.sin(t0))
            b = (r1 * math.cos(t0), y1, r1 * math.sin(t0))
            c = (r1 * math.cos(t1), y1, r1 * math.sin(t1))
            d = (r0 * math.cos(t1), y0, r0 * math.sin(t1))
            m.quad('tyre', a, b, c, d)
    for s in (1, -1):
        ax = np.array([0, s, 0.0])
        y = s * (h - 0.012)
        m.ring('rim', (0, y, 0), ax, 0.235, 0.256, n)
        m.ring('disc', (0, y, 0), ax, 0.0, 0.235, n)
        # five dark aero slots so the spin shows
        for j in range(5):
            t = 2 * math.pi * j / 5
            c = np.array([0.17 * math.cos(t), y + s * 0.002, 0.17 * math.sin(t)])
            m.ring('rim', c, ax, 0.0, 0.028, 12)
        m.ring('rim', (0, y + s * 0.002, 0), ax, 0.0, 0.045, 16)
    return m


# ----------------------------------------------------------------- export

def degenerate_count(mesh):
    """Triangles with no area (a profile typo collapses facets)."""
    bad = 0
    for tris in mesh.tris.values():
        for a, b, c in tris:
            if np.linalg.norm(np.cross(b - a, c - a)) < 1e-10:
                bad += 1
    return bad


XYZ = ''.join(f'<param name="{c}" type="float"/>' for c in 'XYZ')


def write_dae(mesh, path, note):
    effects, materials, bind, prims = [], [], [], []
    pos, nrm = [], []
    for mat, tris in sorted(mesh.tris.items()):
        diffuse, spec, emis = MATERIALS[mat]
        em = emis or (0.0, 0.0, 0.0)
        effects.append(
            f'<effect id="{mat}-fx"><profile_COMMON><technique sid="common"><phong>'
            f'<emission><color>{em[0]} {em[1]} {em[2]} 1</color></emission>'
            f'<ambient><color>{diffuse[0]} {diffuse[1]} {diffuse[2]} 1</color></ambient>'
            f'<diffuse><color>{diffuse[0]} {diffuse[1]} {diffuse[2]} 1</color></diffuse>'
            f'<specular><color>{spec} {spec} {spec} 1</color></specular>'
            f'<shininess><float>{20 + 80 * spec:.0f}</float></shininess>'
            '</phong></technique></profile_COMMON></effect>')
        materials.append(f'<material id="{mat}" name="{mat}"><instance_effect url="#{mat}-fx"/></material>')
        bind.append(f'<instance_material symbol="{mat}" target="#{mat}"/>')
        idx = []
        for a, b, c in tris:
            nv = np.cross(b - a, c - a)
            nl = np.linalg.norm(nv)
            if nl < 1e-12:
                continue
            nrm.append(nv / nl)
            ni = len(nrm) - 1
            for p in (a, b, c):
                pos.append(p)
                idx += [len(pos) - 1, ni]
        prims.append(f'<triangles material="{mat}" count="{len(idx) // 6}">'
                     '<input semantic="VERTEX" source="#mesh-v" offset="0"/>'
                     '<input semantic="NORMAL" source="#mesh-n" offset="1"/>'
                     f'<p>{" ".join(map(str, idx))}</p></triangles>')
    fmt = ' '.join('%.5g' % v for p in pos for v in p)
    fmtn = ' '.join('%.4g' % v for p in nrm for v in p)
    doc = f"""<?xml version="1.0" encoding="utf-8"?>
<!-- GENERATED by scripts/generate_roadster_mesh.py - do not hand-edit. {note} -->
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_effects>{''.join(effects)}</library_effects>
<library_materials>{''.join(materials)}</library_materials>
<library_geometries><geometry id="mesh" name="mesh"><mesh>
<source id="mesh-p"><float_array id="mesh-p-a" count="{3 * len(pos)}">{fmt}</float_array>
<technique_common><accessor source="#mesh-p-a" count="{len(pos)}" stride="3">{XYZ}</accessor>
</technique_common></source>
<vertices id="mesh-v"><input semantic="POSITION" source="#mesh-p"/></vertices>
<source id="mesh-n"><float_array id="mesh-n-a" count="{3 * len(nrm)}">{fmtn}</float_array>
<technique_common><accessor source="#mesh-n-a" count="{len(nrm)}" stride="3">{XYZ}</accessor>
</technique_common></source>
{''.join(prims)}
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node id="body" name="body">
<instance_geometry url="#mesh"><bind_material><technique_common>{''.join(bind)}</technique_common>
</bind_material></instance_geometry>
</node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""
    path.write_text(doc)
    return sum(len(t) for t in mesh.tris.values())


# ----------------------------------------------------------------- preview

def render(mesh, path, view, size=(900, 420), scale=180.0):
    """Orthographic flat-shaded render (painter's algorithm)."""
    from PIL import Image, ImageDraw
    views = {  # (camera forward, camera up)
        'side': ((0, 1, 0), (0, 0, 1)),     # looking at the car's right side
        'top': ((0, 0, -1), (0, 1, 0)),
        'front': ((-1, 0, 0), (0, 0, 1)),
        'rear': ((1, 0, 0), (0, 0, 1)),
        'three_quarter': ((-0.62, 0.62, -0.48), (0, 0, 1)),
        'under': ((0, 0, 1), (0, 1, 0)),
    }
    fwd, up = (np.array(v, float) for v in views[view])
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    light = np.array([0.4, 0.5, 0.75])
    light /= np.linalg.norm(light)
    centre = np.array([0.0, 0.0, 0.55])
    img = Image.new('RGB', size, (40, 40, 46))
    d = ImageDraw.Draw(img)
    items = []
    for mat, tris in mesh.tris.items():
        diffuse, _, emis = MATERIALS[mat]
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            nl = np.linalg.norm(n)
            if nl < 1e-12 or np.dot(n, fwd) >= 0:   # back-face cull
                continue
            n /= nl
            shade = (0.3 + 0.45 * max(0.0, float(np.dot(n, light)))
                     + 0.3 * max(0.0, -float(np.dot(n, fwd))))
            col = emis if emis else tuple(min(1.0, ch * shade) for ch in diffuse)
            pts = [(size[0] / 2 + scale * np.dot(p - centre, right),
                    size[1] / 2 - scale * np.dot(p - centre, up)) for p in (a, b, c)]
            depth = float(np.dot((a + b + c) / 3, fwd))
            items.append((depth, pts, tuple(int(255 * ch) for ch in col)))
    for _, pts, col in sorted(items, key=lambda t: -t[0]):
        d.polygon(pts, fill=col)
    img.save(path)


def preview(body, wheel, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    car = Mesh()
    for mat, tris in body.tris.items():
        car.tris.setdefault(mat, []).extend(tris)
    for wx in wheel_centres_x():
        for wy in (TRACK / 2, -TRACK / 2):
            off = np.array([wx, wy, 0.0])
            for mat, tris in wheel.tris.items():
                car.tris.setdefault(mat, []).extend((a + off, b + off, c + off) for a, b, c in tris)
    car = car.shifted(WHEEL_R)  # back to ground height for the views
    for v in ('side', 'top', 'front', 'rear', 'three_quarter', 'under'):
        render(car, out_dir / f'{v}.png', v)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--preview', type=Path, help='also write PNG views to this directory')
    a = ap.parse_args()
    nozzles = yaml.safe_load((PKG / 'config' / 'nozzles.yaml').read_text())['nozzles']
    body, wheel = build_body(nozzles), build_wheel()
    for m, name in ((body, 'body'), (wheel, 'wheel')):
        bad = degenerate_count(m)
        if bad:
            raise SystemExit(f'{name}: {bad} degenerate triangles')
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    nb = write_dae(body, MESH_DIR / 'roadster_body.dae', 'Frame: chassis link.')
    nw = write_dae(wheel, MESH_DIR / 'roadster_wheel.dae', 'Axis along y.')
    print(f'wrote roadster_body.dae ({nb} triangles), roadster_wheel.dae ({nw} triangles)')
    if a.preview:
        preview(body, wheel, a.preview)
        print(f'previews in {a.preview}')


if __name__ == '__main__':
    main()
