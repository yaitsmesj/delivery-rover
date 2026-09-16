# FILE: src/delivery_rover/delivery_rover/navigator.py
"""Client-side helper for commanding Nav2.

This module starts nothing. Nav2 is brought up by `ros2 launch`; this is
just the API a client (the notebook, the mission node) uses to send goals
to a Nav2 that is already running.

It wraps nav2_simple_commander's BasicNavigator, which is Nav2's own
official Python client — the same library the Nav2 tutorials use. The
only thing added on top is `wait_ready()`.
"""
from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .sim import yaw_to_quat


class Navigator:
    """A thin, poll-friendly face on BasicNavigator."""

    def __init__(self, node_name: str = "navigator_client"):
        # BasicNavigator names its node 'basic_navigator' unless told
        # otherwise, and two nodes with the same name in one ROS graph is
        # undefined behaviour — goals and feedback get crossed and
        # navigation fails in ways that look like tuning problems. Every
        # Navigator in this project therefore gets its own name.
        self._nav = BasicNavigator(node_name=node_name)

    # -- readiness --------------------------------------------------------
    def wait_ready(self, timeout: float = 60.0) -> bool:
        """Ready means two things.

        First, Nav2's own definition: every lifecycle server is ACTIVE.
        `waitUntilNav2Active` is the official check — with no AMCL in this
        stack, map_server plays the localizer's part.

        Second, ours: both costmaps have published at least once. ACTIVE
        alone is a trap — a goal sent before the costmaps have digested
        their first scan dies with "plan has 0 poses" / "Pose Goes Off
        Grid". This is the single most useful line in the file.
        """
        self._nav.waitUntilNav2Active(localizer="map_server")
        return self._wait_costmaps(time.time() + timeout)

    def _wait_costmaps(self, deadline: float) -> bool:
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        seen = set()
        subs = [
            self._nav.create_subscription(
                OccupancyGrid, topic, lambda m, t=topic: seen.add(t), 3)
            for topic in ("/local_costmap/costmap", "/global_costmap/costmap")
        ]
        try:
            while len(seen) < 2 and time.time() < deadline:
                rclpy.spin_once(self._nav, timeout_sec=0.25)
            return len(seen) == 2
        finally:
            for sub in subs:
                self._nav.destroy_subscription(sub)

    # -- goals ------------------------------------------------------------
    def go_to(self, x: float, y: float, yaw: float = 0.0) -> None:
        """Send a NavigateToPose goal and return immediately."""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self._nav.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        _, _, qz, qw = yaw_to_quat(yaw)
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self._nav.goToPose(goal)

    def is_done(self) -> bool:
        return self._nav.isTaskComplete()

    def succeeded(self) -> bool:
        return self._nav.getResult() == TaskResult.SUCCEEDED

    def cancel(self) -> None:
        self._nav.cancelTask()

    def distance_remaining(self) -> float:
        fb = self._nav.getFeedback()
        return round(fb.distance_remaining, 2) if fb else math.nan

    def go_to_blocking(self, x: float, y: float, yaw: float = 0.0,
                       timeout: float = 120.0) -> bool:
        """Notebook convenience: drive there, wait, report success."""
        self.go_to(x, y, yaw)
        deadline = time.time() + timeout
        while not self.is_done():
            if time.time() > deadline:
                self.cancel()
                return False
            time.sleep(0.25)
        return self.succeeded()

    def destroy(self) -> None:
        self._nav.destroy_node()
