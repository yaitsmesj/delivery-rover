# FILE: src/delivery_rover/delivery_rover/world.py
"""The depot world — one source of truth for the map, the sim, and the mission.

The world is data: a floor plan made of rectangles plus a set of named
locations. Everything else (the Nav2 map files, the simulated lidar, the
mission goals) is derived from it, so the map in the costmap and the walls
the sim robot collides with can never disagree.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

RESOLUTION = 0.05          # metres per cell
SIZE_X, SIZE_Y = 12.0, 8.0  # depot footprint in metres

# Rectangles (x_min, y_min, x_max, y_max) in metres — walls and shelving.
OBSTACLES = [
    # outer walls, 10 cm thick
    (0.0, 0.0, SIZE_X, 0.1), (0.0, SIZE_Y - 0.1, SIZE_X, SIZE_Y),
    (0.0, 0.0, 0.1, SIZE_Y), (SIZE_X - 0.1, 0.0, SIZE_X, SIZE_Y),
    # shelving units
    (3.0, 2.0, 3.6, 6.0),
    (6.0, 0.0, 6.6, 4.0),
    (9.0, 3.5, 9.6, 8.0),
]

# Named locations: (x, y, yaw in radians). yaw = direction the rover faces.
LOCATIONS = {
    "home":    (1.0, 1.0, 0.0),
    "pickup":  (5.0, 6.8, -math.pi / 2),
    "dropoff": (11.0, 1.2, 0.0),
}


def occupancy_grid() -> np.ndarray:
    """The floor plan as a grid: 0 = free, 1 = occupied. grid[row, col],
    row 0 at y=0 (the map's bottom edge)."""
    nx, ny = int(SIZE_X / RESOLUTION), int(SIZE_Y / RESOLUTION)
    grid = np.zeros((ny, nx), dtype=np.uint8)
    for (x0, y0, x1, y1) in OBSTACLES:
        c0, c1 = int(x0 / RESOLUTION), int(np.ceil(x1 / RESOLUTION))
        r0, r1 = int(y0 / RESOLUTION), int(np.ceil(y1 / RESOLUTION))
        grid[r0:r1, c0:c1] = 1
    return grid


def is_occupied(x: float, y: float) -> bool:
    """Point-in-wall test in metres, used by the sim for collision."""
    if not (0.0 <= x < SIZE_X and 0.0 <= y < SIZE_Y):
        return True
    grid = _cached_grid()
    return bool(grid[int(y / RESOLUTION), int(x / RESOLUTION)])


_GRID = None


def _cached_grid() -> np.ndarray:
    global _GRID
    if _GRID is None:
        _GRID = occupancy_grid()
    return _GRID


def save_map(out_dir: str | Path) -> Path:
    """Write the Nav2 map pair (depot.pgm + depot.yaml). Returns yaml path.

    PGM convention: white (254) = free, black (0) = occupied, and row 0 of
    the image is the TOP of the map — so the grid is flipped vertically.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    grid = occupancy_grid()
    img = np.where(np.flipud(grid) == 1, 0, 254).astype(np.uint8)
    pgm = out / "depot.pgm"
    with open(pgm, "wb") as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
        f.write(img.tobytes())
    meta = {
        "image": "depot.pgm",
        "mode": "trinary",
        "resolution": RESOLUTION,
        "origin": [0.0, 0.0, 0.0],   # map (0,0) is the world's (0,0)
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    map_yaml = out / "depot.yaml"
    map_yaml.write_text(yaml.dump(meta, sort_keys=False))
    return map_yaml