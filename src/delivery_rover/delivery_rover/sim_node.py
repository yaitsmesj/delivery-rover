# FILE: src/delivery_rover/delivery_rover/sim_node.py
"""Entry point for the simulated base: `ros2 run delivery_rover sim`.

Every ROS 2 Python node looks like this — declare parameters, construct
the node, spin it, shut down on Ctrl-C. The launch system starts it; the
launch system stops it.
"""
from __future__ import annotations

import rclpy

from .sim import DiffDriveSim
from .world import LOCATIONS


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSim()

    # A launch-time override: ros2 run delivery_rover sim --ros-args -p start:=pickup
    node.declare_parameter("start", "home")
    start = node.get_parameter("start").value
    if start in LOCATIONS and start != "home":
        node.x, node.y, node.yaw = LOCATIONS[start]

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()