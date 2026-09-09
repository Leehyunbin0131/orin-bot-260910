#!/usr/bin/env python3
"""Texture patch generator script for Visual Odometry and ArUco dock targets.

    python3 generate_textures.py
"""

import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'materials', 'textures')

rng = np.random.default_rng(20260801)

# Docking station target (ArUco DICT_4X4_50)
ARUCO_DICT = 0
ARUCO_IDS = (1, 0, 2)


def save(img, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    img.save(path)
    print('wrote', path)


def speckle(base_rgb, size=512, amount=38, blobs=900):
    """Base color + random speckles for feature detection."""
    arr = np.full((size, size, 3), base_rgb, dtype=np.int16)
    arr += rng.integers(-amount, amount, (size, size, 3), dtype=np.int16)
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    for _ in range(blobs):
        x, y = rng.integers(0, size, 2)
        r = rng.integers(1, 4)
        c = tuple(int(v) for v in np.clip(
            np.array(base_rgb) + rng.integers(-amount * 2, amount * 2, 3), 0, 255))
        d.ellipse([x - r, y - r, x + r, y + r], fill=c)
    return img


def brick_wall(size=512):
    """Brick wall texture with high-contrast line corners."""
    img = speckle((176, 122, 98), size, amount=22, blobs=500)
    d = ImageDraw.Draw(img)
    rows, bh = 12, size // 12
    for r in range(rows + 1):
        y = r * bh
        d.line([(0, y), (size, y)], fill=(80, 75, 70), width=3)
    for r in range(rows):
        y0, y1 = r * bh, (r + 1) * bh
        offset = (bh * 2) if (r % 2) else 0
        for c in range(8):
            x = c * (size // 4) + offset
            d.line([(x, y0), (x, y1)], fill=(80, 75, 70), width=2)
    return img


def tiles(size=512):
    """Floor tiles with subtle color variance per cell."""
    img = Image.new('RGB', (size, size), (150, 150, 155))
    d = ImageDraw.Draw(img)
    n, cell = 8, size // 8
    for r in range(n):
        for c in range(n):
            x0, y0 = c * cell, r * cell
            col = tuple(int(v) for v in np.clip(
                np.array([165, 162, 158]) + rng.integers(-15, 15, 3), 0, 255))
            d.rectangle([x0 + 2, y0 + 2, x0 + cell - 2, y0 + cell - 2], fill=col)
            d.rectangle([x0, y0, x0 + cell, y0 + cell], outline=(100, 100, 105), width=2)
    return img


def aruco_marker(marker_id=0, dictionary=ARUCO_DICT):
    """Docking station target marker generated via OpenCV ArUco dictionary."""
    import cv2
    aruco_dict = cv2.aruco.Dictionary_get(dictionary)
    pix = cv2.aruco.drawMarker(aruco_dict, marker_id, 360)
    canvas = np.full((504, 504), 255, dtype=np.uint8)
    canvas[72:432, 72:432] = pix
    rgb = np.stack([canvas] * 3, axis=-1)
    return Image.fromarray(rgb)


def poster(size=512, seed_shapes=26):
    """Wall poster texture with unique geometric patterns per index."""
    img = speckle((230, 225, 215), size, amount=12, blobs=300)
    d = ImageDraw.Draw(img)
    colors = [(200, 60, 50), (40, 110, 180), (40, 150, 90), (220, 160, 40), (120, 70, 150)]
    for i in range(5):
        c = colors[i % len(colors)]
        x0 = 40 + (i * 85) % (size - 120)
        y0 = 60 + (i * 95) % (size - 140)
        shape = i % 3
        if shape == 0:
            d.rectangle([x0, y0, x0 + 70, y0 + 100], fill=c)
        elif shape == 1:
            d.ellipse([x0, y0, x0 + 80, y0 + 80], fill=c)
        else:
            d.polygon([(x0 + 40, y0), (x0 + 80, y0 + 80), (x0, y0 + 80)], fill=c)
    return img


N_WALL_SEGMENTS = 28
N_POSTERS = 8

N_MAZE_SEGMENTS = 80
MAZE_SEED = 20260803

N_HALL_SEGMENTS = 100
HALL_SEED = 20260804

N_OFFICE_SEGMENTS = 66
OFFICE_SEED = 20260805


def mark(img, idx):
    """Apply distinct geometric landmark marks per wall segment index."""
    d = ImageDraw.Draw(img)
    w, h = img.size
    hue = (idx * 0.618033988749895) % 1.0
    h_i = int(hue * 6)
    f = hue * 6 - h_i
    q = int(255 * (1 - f))
    t = int(255 * f)
    if h_i == 0:
        rgb = (255, t, 0)
    elif h_i == 1:
        rgb = (q, 255, 0)
    elif h_i == 2:
        rgb = (0, 255, t)
    elif h_i == 3:
        rgb = (0, q, 255)
    elif h_i == 4:
        rgb = (t, 0, 255)
    else:
        rgb = (255, 0, q)

    shape = idx % 4
    cx, cy = w // 2, h // 2
    r = 90
    if shape == 0:
        d.rectangle([cx - r, cy - r, cx + r, cy + r], fill=rgb, outline=(30, 30, 30), width=6)
    elif shape == 1:
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=rgb, outline=(30, 30, 30), width=6)
    elif shape == 2:
        pts = [(cx, cy - r), (cx + r, cy + r), (cx - r, cy + r)]
        d.polygon(pts, fill=rgb, outline=(30, 30, 30))
    else:
        d.line([(cx - r, cy - r), (cx + r, cy + r)], fill=rgb, width=16)
        d.line([(cx - r, cy + r), (cx + r, cy - r)], fill=rgb, width=16)

    for b in range(4):
        bx = 40 + b * 25
        if (idx >> b) & 1:
            d.rectangle([bx, h - 50, bx + 18, h - 20], fill=(20, 20, 20))
        else:
            d.rectangle([bx, h - 50, bx + 18, h - 20], outline=(20, 20, 20), width=3)
    return img


def main_maze():
    global rng
    rng = np.random.default_rng(MAZE_SEED)
    for i in range(N_MAZE_SEGMENTS):
        save(mark(brick_wall(), i + 100), 'maze_%03d.png' % i)
    print('Generated %d maze wall textures' % N_MAZE_SEGMENTS)


def office_wall(idx, size=512):
    """Low-contrast office wall texture."""
    img = speckle((225, 223, 218), size, amount=8, blobs=200)
    arr = np.array(img, dtype=np.int16)
    arr += rng.integers(-4, 5, (size, size, 3), dtype=np.int16)
    for r in range(size):
        arr[r] = np.clip(arr[r] - int(r * 12 / size), 0, 255)
    img = Image.fromarray(arr.astype(np.uint8))
    d = ImageDraw.Draw(img)
    d.rectangle([0, size - 35, size, size], fill=(160, 155, 148))
    d.line([(0, size - 35), (size, size - 35)], fill=(120, 115, 108), width=2)
    cx, cy = 100 + (idx * 37) % (size - 200), 120 + (idx * 53) % (size - 240)
    d.rectangle([cx, cy, cx + 18, cy + 28], fill=(180, 175, 168), outline=(140, 135, 128))
    return img


def main_office():
    global rng
    rng = np.random.default_rng(OFFICE_SEED)
    for i in range(N_OFFICE_SEGMENTS):
        save(office_wall(i), 'office_%03d.png' % i)
    print('Generated %d office wall textures (low contrast)' % N_OFFICE_SEGMENTS)


def main_hall():
    global rng
    rng = np.random.default_rng(HALL_SEED)
    for i in range(N_HALL_SEGMENTS):
        save(mark(brick_wall(), i + 200), 'hall_%03d.png' % i)
    print('Generated %d hall wall textures' % N_HALL_SEGMENTS)


MODES = {'maze': main_maze, 'hall': main_hall, 'office': main_office}


def main():
    if len(sys.argv) > 1:
        for m in sys.argv[1:]:
            if m in MODES:
                MODES[m]()
            else:
                sys.exit('Unknown mode: %s (valid: %s)' % (m, list(MODES.keys())))
        return

    for i in range(N_WALL_SEGMENTS):
        save(mark(brick_wall(), i), 'wall_%02d.png' % i)
    save(tiles(), 'floor.png')

    for i in range(N_POSTERS):
        save(poster(seed_shapes=i), 'poster_%d.png' % i)

    save(speckle((140, 100, 60), blobs=600), 'crate.png')
    for m_id in ARUCO_IDS:
        save(aruco_marker(m_id), 'dock_marker_%d.png' % m_id)

    save(speckle((60, 70, 90), blobs=800), 'dock_body.png')


if __name__ == '__main__':
    main()
