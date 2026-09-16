# FILE: src/delivery_rover/delivery_rover/make_map.py
"""Render the depot into Nav2 map files: `ros2 run delivery_rover make_map`.

Run this once, into the package's source tree, then commit the result —
exactly what you would do with a map produced by slam_toolbox on a real
robot. map_server loads the installed copy at runtime.

    ros2 run delivery_rover make_map src/delivery_rover/config/map
"""
from __future__ import annotations

import sys
from pathlib import Path

from .world import save_map


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    out = Path(argv[0]) if argv else Path("config/map")
    path = save_map(out)
    print(f"wrote {path} and {path.with_suffix('.pgm')}")
