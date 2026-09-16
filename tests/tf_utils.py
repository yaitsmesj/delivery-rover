# FILE: tests/tf_utils.py
"""TFHelper — the notebook's window into the transform tree (Ch 3)."""
from __future__ import annotations

import math
import time

from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TFHelper:
    """Attach to any spinning node; the listener fills the buffer in the
    background. where_is() answers "where does the robot think it is?"."""

    def __init__(self, node: Node):
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, node)

    def where_is(self, target: str = "map", source: str = "base_link"):
        t = self.buffer.lookup_transform(target, source, Time()).transform
        yaw = 2.0 * math.atan2(t.rotation.z, t.rotation.w)
        return (round(t.translation.x, 3), round(t.translation.y, 3),
                round(math.degrees(yaw), 1))

    def can_see(self, target: str = "map", source: str = "base_link") -> bool:
        return self.buffer.can_transform(target, source, Time())

    def wait_for(self, target: str = "map", source: str = "base_link",
                 timeout: float = 15.0) -> bool:
        """Block until the chain target<-source exists, or give up.

        A client that has just started has not discovered the rest of the
        graph yet — with the robot in other processes that handshake takes
        a second or two, unlike an in-process call. Waiting for the data
        beats sleeping a guessed number of seconds.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.can_see(target, source):
                return True
            time.sleep(0.1)
        return False