#!/usr/bin/env python3
"""
generate_map.py — Generates a Nav2-compatible occupancy map from datacenter_layout.json.

Outputs:
    maps/datacenter_map.pgm   (PGM grayscale image)
    maps/datacenter_map.yaml  (Nav2 map metadata)

Resolution: 0.05 m/pixel
Conventions: 255 = free, 0 = occupied, 205 = unknown
"""

import json
import math
import os
import struct
import sys

RESOLUTION = 0.05   # meters per pixel
FREE       = 255
OCCUPIED   = 0
UNKNOWN    = 205
WALL_COLOR = 0      # black
RACK_COLOR = 0      # black


def world_to_pixel(x, y, width_px, height_px, resolution):
    """Convert world coordinates (m) to pixel indices (col, row)."""
    col = int(x / resolution)
    row = height_px - 1 - int(y / resolution)   # flip y-axis
    return col, row


def draw_box(grid, cx, cy, w, d, width_px, height_px, resolution, color):
    """Fill a box (center cx, cy, width w, depth d) with color."""
    x_min = cx - w / 2
    x_max = cx + w / 2
    y_min = cy - d / 2
    y_max = cy + d / 2

    col_min = max(0, int(math.floor(x_min / resolution)))
    col_max = min(width_px - 1, int(math.ceil(x_max / resolution)))
    row_min = max(0, height_px - 1 - int(math.ceil(y_max / resolution)))
    row_max = min(height_px - 1, height_px - 1 - int(math.floor(y_min / resolution)))

    for row in range(row_min, row_max + 1):
        for col in range(col_min, col_max + 1):
            grid[row][col] = color


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root  = os.path.join(script_dir, '..', '..')
    layout_path = os.path.join(repo_root, 'config', 'datacenter_layout.json')
    maps_dir    = os.path.join(script_dir, '..', 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    with open(layout_path, 'r') as f:
        layout = json.load(f)

    room        = layout['room']
    wall_t      = room['wall_thickness_m']
    room_w      = room['width_m']
    room_d      = room['depth_m']
    total_w     = room_w + 2 * wall_t
    total_d     = room_d + 2 * wall_t

    width_px  = int(math.ceil(total_w / RESOLUTION))
    height_px = int(math.ceil(total_d / RESOLUTION))

    # Initialize grid: all free
    grid = [[FREE] * width_px for _ in range(height_px)]

    # ── Walls ─────────────────────────────────────────────────────────────
    # Offset: room starts at (wall_t, wall_t) in map coords
    wt_px = int(math.ceil(wall_t / RESOLUTION))

    # South wall
    for row in range(height_px - wt_px, height_px):
        for col in range(width_px):
            grid[row][col] = WALL_COLOR
    # North wall
    for row in range(0, wt_px):
        for col in range(width_px):
            grid[row][col] = WALL_COLOR
    # West wall
    for row in range(height_px):
        for col in range(0, wt_px):
            grid[row][col] = WALL_COLOR
    # East wall
    for row in range(height_px):
        for col in range(width_px - wt_px, width_px):
            grid[row][col] = WALL_COLOR

    # ── Racks ─────────────────────────────────────────────────────────────
    for rack in layout['racks']:
        # Rack coords in layout are in room frame (0,0 = room SW corner)
        # Map frame: (0,0) = map SW corner = room SW corner - wall_t
        cx = rack['x'] + wall_t
        cy = rack['y'] + wall_t
        draw_box(grid, cx, cy, rack['width_m'], rack['depth_m'],
                 width_px, height_px, RESOLUTION, RACK_COLOR)

    # ── Write PGM ─────────────────────────────────────────────────────────
    pgm_path = os.path.join(maps_dir, 'datacenter_map.pgm')
    with open(pgm_path, 'wb') as f:
        header = f'P5\n{width_px} {height_px}\n255\n'
        f.write(header.encode('ascii'))
        for row in grid:
            f.write(bytes(row))

    # ── Write YAML ────────────────────────────────────────────────────────
    # Map origin: world position (m) of the bottom-left pixel
    origin_x = -wall_t
    origin_y = -wall_t

    yaml_path = os.path.join(maps_dir, 'datacenter_map.yaml')
    yaml_content = f"""\
image: datacenter_map.pgm
resolution: {RESOLUTION}
origin: [{origin_x}, {origin_y}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    print(f'Map generated: {width_px}x{height_px} px @ {RESOLUTION} m/px')
    print(f'  PGM  → {pgm_path}')
    print(f'  YAML → {yaml_path}')


if __name__ == '__main__':
    main()
