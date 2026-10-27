#!/usr/bin/env python3
"""Rasterize the box collisions of the static "house" model into a Nav2 map (pgm + yaml).

Usage: world_to_map.py <world.sdf> <output_dir> [map_name]
Only boxes rotated about z are supported (all house geometry is axis-aligned or yawed).
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

RESOLUTION = 0.05     # m/cell
MARGIN = 0.5          # m of unknown space around the house
MIN_HEIGHT = 0.03     # boxes lower than this are not obstacles (e.g. thresholds)
FREE, OCCUPIED, UNKNOWN = 254, 0, 205


def load_boxes(world_file):
    root = ET.parse(world_file).getroot()
    house = root.find(".//model[@name='house']")
    boxes = []
    for col in house.iter('collision'):
        size = col.find('geometry/box/size')
        if size is None:
            continue
        sx, sy, sz = map(float, size.text.split())
        pose = col.find('pose')
        x, y, z, _, _, yaw = map(float, pose.text.split()) if pose is not None else (0,) * 6
        if z + sz / 2 < MIN_HEIGHT:
            continue
        boxes.append((x, y, sx, sy, yaw))
    return boxes


def main():
    world_file, out_dir = sys.argv[1], sys.argv[2]
    name = sys.argv[3] if len(sys.argv) > 3 else 'house'
    boxes = load_boxes(world_file)

    # Footprint of every box corner -> map bounds
    corners = []
    for x, y, sx, sy, yaw in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        for dx, dy in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (sx / 2, -sy / 2), (-sx / 2, -sy / 2)):
            corners.append((x + c * dx - s * dy, y + s * dx + c * dy))
    min_x = min(p[0] for p in corners)
    min_y = min(p[1] for p in corners)
    max_x = max(p[0] for p in corners)
    max_y = max(p[1] for p in corners)

    ox, oy = min_x - MARGIN, min_y - MARGIN
    width = int(math.ceil((max_x - min_x + 2 * MARGIN) / RESOLUTION))
    height = int(math.ceil((max_y - min_y + 2 * MARGIN) / RESOLUTION))

    # Inside the outer walls is free, outside is unknown
    grid = [[UNKNOWN] * width for _ in range(height)]
    for row in range(height):
        wy = oy + (row + 0.5) * RESOLUTION
        for col in range(width):
            wx = ox + (col + 0.5) * RESOLUTION
            if min_x <= wx <= max_x and min_y <= wy <= max_y:
                grid[row][col] = FREE

    for x, y, sx, sy, yaw in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        for row in range(height):
            wy = oy + (row + 0.5) * RESOLUTION
            for col in range(width):
                wx = ox + (col + 0.5) * RESOLUTION
                lx = c * (wx - x) + s * (wy - y)
                ly = -s * (wx - x) + c * (wy - y)
                if abs(lx) <= sx / 2 and abs(ly) <= sy / 2:
                    grid[row][col] = OCCUPIED

    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, name + '.pgm'), 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        for row in reversed(range(height)):  # pgm top row = max y
            f.write(bytes(grid[row]))
    with open(os.path.join(out_dir, name + '.yaml'), 'w') as f:
        f.write(f'image: {name}.pgm\n'
                f'mode: trinary\n'
                f'resolution: {RESOLUTION}\n'
                f'origin: [{ox:.3f}, {oy:.3f}, 0.0]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.25\n')
    print(f'{name}: {width}x{height} cells, origin ({ox:.3f}, {oy:.3f}), {len(boxes)} boxes')


if __name__ == '__main__':
    main()
