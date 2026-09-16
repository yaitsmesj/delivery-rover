# FILE: src/delivery_rover/delivery_rover/vision_node.py
"""Entry point for perception: `ros2 run delivery_rover vision`.

Parameters (set them in perception.launch.py, or with --ros-args -p):
    source      camera index as a string ("0"), or a path to a video file
    model       Ultralytics weights name, default yolo11n.pt
    device      "" picks automatically (mps / cuda / cpu)
    rate_hz     frames handed to the detector per second
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from .vision import RobotVisionBrain


def main(args=None):
    rclpy.init(args=args)

    # Read parameters off a throwaway node, then build the real one with
    # them — the vision node's constructor opens the camera, so it needs
    # its settings up front.
    cfg = Node("vision_config")
    cfg.declare_parameter("source", "0")
    cfg.declare_parameter("model", "yolo11n.pt")
    cfg.declare_parameter("device", "")
    cfg.declare_parameter("rate_hz", 5.0)
    source = cfg.get_parameter("source").value
    model = cfg.get_parameter("model").value
    device = cfg.get_parameter("device").value or None
    rate_hz = cfg.get_parameter("rate_hz").value
    cfg.destroy_node()

    # "0" means camera index 0; anything else is a file path.
    src = int(source) if str(source).isdigit() else str(source)

    node = RobotVisionBrain(source=src, model_name=model,
                            device=device, rate_hz=rate_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()