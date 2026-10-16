#!/usr/bin/env python3
"""generate_puff_texture.py - soft round particle texture for the plume effects.

Writes roadster_description/models/fx/puff.png: 64 x 64 RGBA, white, with a
smooth radial alpha falloff, so Gazebo particles read as soft gas puffs instead
of hard squares. Pure-stdlib PNG writer (no numpy / cv2: the venv's NumPy 2
breaks cv2 on this machine, CLAUDE.md).

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


def main():
    rows = []
    c = (N - 1) / 2
    for y in range(N):
        row = []
        for x in range(N):
            r = math.hypot(x - c, y - c) / c
            a = max(0.0, 1.0 - r) ** 1.6  # soft edge, dense core
            row += [255, 255, 255, int(round(255 * a))]
        rows.append(row)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'puff.png').write_bytes(png(N, N, rows))
    print(f'wrote {OUT / "puff.png"}')


if __name__ == '__main__':
    main()
