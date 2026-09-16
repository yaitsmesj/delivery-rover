"""Build a test video for perception, so M4 needs no webcam.

    python3 tests/make_feed.py            # writes assets/feed.mp4
    python3 tests/make_feed.py out.mp4

Uses the two sample photos that ship inside the `ultralytics` package —
already in your environment, nothing to download. It pans across each one
so consecutive frames differ, which is what a real feed looks like to the
detector. The video contains a person, a bus and a tie; the vision node
loops the file forever, so detections keep arriving.

    ros2 launch delivery_rover bringup.launch.py mission:=false \\
        source:=assets/feed.mp4
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import ultralytics

W, H, FPS, SECONDS_EACH = 640, 480, 15, 4


def pan(img, out, frames):
    """Write `frames` frames panning left-to-right across one image."""
    h, w = img.shape[:2]
    scale = H / h                              # fit the height, crop width
    img = cv2.resize(img, (int(w * scale), H))
    travel = max(img.shape[1] - W, 0)
    for i in range(frames):
        x = int(travel * i / max(frames - 1, 1))
        crop = img[:, x:x + W]
        if crop.shape[1] < W:                  # narrower than the frame: pad
            crop = np.pad(crop, ((0, 0), (0, W - crop.shape[1]), (0, 0)))
        out.write(crop)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    dest = Path(argv[0]) if argv else Path("assets/feed.mp4")
    dest.parent.mkdir(parents=True, exist_ok=True)

    assets = Path(ultralytics.__file__).parent / "assets"
    photos = sorted(assets.glob("*.jpg"))
    if not photos:
        raise SystemExit(f"no sample photos found in {assets}")

    writer = cv2.VideoWriter(
        str(dest), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    for photo in photos:
        pan(cv2.imread(str(photo)), writer, FPS * SECONDS_EACH)
    writer.release()

    print(f"wrote {dest} — {len(photos) * SECONDS_EACH}s, {FPS} fps, "
          f"from {', '.join(p.name for p in photos)}")


if __name__ == "__main__":
    main()