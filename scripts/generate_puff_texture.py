#!/usr/bin/env python3
"""generate_puff_texture.py - soft round particle textures for the plume effects.

Writes into roadster_description/models/fx/, each 64 x 64 RGBA, white, with a
smooth radial alpha falloff, so Gazebo particles read as gas instead of hard
squares:
  puff.png  dense core, soft edge: the fan exhaust's sparse puffs
  jet.png   faint Gaussian (peak alpha 0.25): the nozzle jets, where a dozen
            or more particles overlap at every point, so the sum builds a
            bright narrow core while the edges stay soft (a denser texture
            saturates into a hard-edged white rod)
Pure-stdlib PNG writer (no numpy / cv2: the venv's NumPy 2 breaks cv2 on this
machine, CLAUDE.md).

  python3 scripts/generate_puff_texture.py
"""
import math
import struct
import zlib
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / 'src' / 'roadster_description' / 'models' / 'fx'
N = 64


def png(width, height, rows):
    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data +
                struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))
    raw = b''.join(b'\x00' + bytes(r) for r in rows)
    return (b'\x89PNG\r\n\x1a\n' +
            chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 6, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def puff_alpha(r):
    return max(0.0, 1.0 - r) ** 1.6  # soft edge, dense core


def jet_alpha(r):
    # Gaussian (sigma 0.35 of the radius), pulled to zero at the rim
    return 0.25 * math.exp(-0.5 * (r / 0.35) ** 2) * max(0.0, 1.0 - r ** 4)


def texture(alpha):
    rows = []
    c = (N - 1) / 2
    for y in range(N):
        row = []
        for x in range(N):
            r = math.hypot(x - c, y - c) / c
            row += [255, 255, 255, int(round(255 * alpha(r)))]
        rows.append(row)
    return png(N, N, rows)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name, alpha in (('puff.png', puff_alpha), ('jet.png', jet_alpha)):
        (OUT / name).write_bytes(texture(alpha))
        print(f'wrote {OUT / name}')


if __name__ == '__main__':
    main()
