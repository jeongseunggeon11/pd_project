#!/usr/bin/env python3
"""Offline check that the patrol waypoints see every free map cell and every fire candidate.

A cell is seen from a waypoint (360 deg scan) if it is within max_range and the straight
line to it crosses no occupied/unknown cell (furniture is taller than the thermal camera).

Usage: check_coverage.py <map.yaml> <waypoints.yaml> <fire_sim.yaml> [uncovered.png]
"""
import math
import os
import sys

import numpy as np
import yaml

MAX_RANGE = 6.0  # [m] detection range verified in milestone 3
FIRE_RADIUS = 0.15  # [m] a fire center needs this much free space around it
FREE = 254


def load_map(map_yaml):
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(map_yaml), meta['image']), 'rb') as f:
        _, size, _, data = f.read().split(b'\n', 3)
    w, h = map(int, size.split())
    img = np.frombuffer(data, np.uint8).reshape(h, w)[::-1]  # row 0 = min y
    return img, meta['resolution'], meta['origin'][0], meta['origin'][1]


def visible_from(img, res, ox, oy, x, y):
    """Boolean grid of cells visible from (x, y), by ray casting to every free cell."""
    h, w = img.shape
    seen = np.zeros_like(img, dtype=bool)
    rows, cols = np.nonzero(img == FREE)
    step = res / 2
    for r, c in zip(rows, cols):
        tx, ty = ox + (c + 0.5) * res, oy + (r + 0.5) * res
        d = math.hypot(tx - x, ty - y)
        if d > MAX_RANGE:
            continue
        n = max(1, int(d / step))
        ok = True
        for i in range(1, n):
            px = x + (tx - x) * i / n
            py = y + (ty - y) * i / n
            if img[int((py - oy) / res), int((px - ox) / res)] != FREE:
                ok = False
                break
        seen[r, c] = ok
    return seen


def main():
    map_yaml, wp_yaml, fire_yaml = sys.argv[1:4]
    img, res, ox, oy = load_map(map_yaml)
    with open(wp_yaml) as f:
        flat = yaml.safe_load(f)['mission_node']['ros__parameters']['waypoints']
    waypoints = list(zip(flat[0::2], flat[1::2]))
    with open(fire_yaml) as f:
        flat = yaml.safe_load(f)['fire_generator']['ros__parameters']['candidates']
    candidates = list(zip(flat[0::2], flat[1::2]))

    seen = np.zeros_like(img, dtype=bool)
    per_wp = []
    for x, y in waypoints:
        if img[int((y - oy) / res), int((x - ox) / res)] != FREE:
            print(f'WARNING: waypoint ({x}, {y}) is not in free space')
        s = visible_from(img, res, ox, oy, x, y)
        per_wp.append(s)
        seen |= s

    free = img == FREE
    print(f'Free cells seen: {seen.sum()}/{free.sum()} ({100.0 * seen.sum() / free.sum():.2f} %)')

    # Cells where a fire fits: every cell within FIRE_RADIUS is free
    k = int(round(FIRE_RADIUS / res))
    placeable = free.copy()
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            if dr * dr + dc * dc <= k * k:
                placeable &= np.roll(np.roll(free, dr, axis=0), dc, axis=1)
    unseen = placeable & ~seen
    print(f'Fire-placeable cells seen: {(placeable & seen).sum()}/{placeable.sum()}')
    for r, c in zip(*np.nonzero(unseen)):
        print(f'  unseen fire-placeable cell ({ox + (c + 0.5) * res:.2f}, {oy + (r + 0.5) * res:.2f})')
    all_ok = True
    for cx, cy in candidates:
        r, c = int((cy - oy) / res), int((cx - ox) / res)
        by = [i + 1 for i, s in enumerate(per_wp) if s[r, c]]
        all_ok &= bool(by)
        print(f'  candidate ({cx:.1f}, {cy:.1f}): seen from waypoints {by if by else "NONE"}')
    print('All candidates covered' if all_ok else 'SOME CANDIDATES NOT COVERED')

    if len(sys.argv) > 4:
        # grey = obstacle/unknown, white = seen, red = free but unseen, blue = waypoint
        rgb = np.stack([img] * 3, axis=-1).astype(np.uint8)
        rgb[free & seen] = (255, 255, 255)
        rgb[free & ~seen] = (230, 30, 30)
        for x, y in waypoints:
            r, c = int((y - oy) / res), int((x - ox) / res)
            rgb[max(r - 2, 0):r + 3, max(c - 2, 0):c + 3] = (30, 30, 230)
        rgb = rgb[::-1]
        with open(sys.argv[4], 'wb') as f:
            f.write(f'P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n'.encode())
            f.write(rgb.tobytes())


if __name__ == '__main__':
    main()
