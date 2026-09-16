# FILE: tests/cockpit.py
"""Test/scripting helper: a spinning ROS client.

Why this exists: a ROS node's callbacks only fire while something spins
it. A node's own `main()` calls `rclpy.spin()` and blocks forever — fine
for a process the launch system owns, useless in a notebook, where you
want the prompt back after every cell.

So an interactive client spins on a background thread instead. That is
the only thing this module does, and it is client-side only: nothing the
robot runs ever imports it.

This is NOT part of the robot. It exists so verification scripts can
subscribe, look up transforms and send goals from outside. A human does
the same jobs with `ros2 topic echo`, `ros2 run tf2_ros tf2_echo` and
`ros2 action send_goal` — this is just those, in a loop, with asserts.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf_utils import TFHelper

_session = None


class Cockpit:
    """One rclpy session per notebook kernel, spun in the background."""

    def __init__(self, name: str = "cockpit"):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(name)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self._extra: list[Node] = []
        self._thread = threading.Thread(
            target=self.executor.spin, daemon=True, name="ros-client-spin")
        self._thread.start()
        self.tf = TFHelper(self.node)

    def add(self, node: Node) -> Node:
        """Spin an extra client node (e.g. a Navigator's internal node)."""
        self._extra.append(node)
        self.executor.add_node(node)
        return node

    def wait_until(self, predicate, timeout: float = 15.0,
                   poll: float = 0.1) -> bool:
        """Wait for something to become true, instead of sleeping blindly.

        Use it after subscribing: `cp.wait_until(lambda: len(scans) > 0)`.
        Discovery between processes is not instantaneous, so a fixed sleep
        is either too short (flaky) or too long (slow).
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(poll)
        return False

    def shutdown(self) -> None:
        for node in self._extra:
            self.executor.remove_node(node)
            node.destroy_node()
        self._extra.clear()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown(timeout_sec=2.0)
        if rclpy.ok():
            rclpy.shutdown()
        global _session
        _session = None


def get_cockpit() -> Cockpit:
    """The one session for this kernel, created on first use."""
    global _session
    if _session is None:
        _session = Cockpit()
    return _session
