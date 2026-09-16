# FILE: src/delivery_rover/delivery_rover/detections.py
"""Listening to perception from another process.

The vision node publishes `/robot/ai/detections`; anything that wants to
know what the robot can see subscribes to it. That includes the mission
node and the notebook — neither of them holds a reference to the vision
node, because it is a separate process on the other side of the graph.

Detections expire. If the vision node dies, its last message would
otherwise sit in memory forever and the mission would believe it can
still see the cargo. `max_age` is what makes "I saw it" mean "I see it".
"""
from __future__ import annotations

from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

CONF_THRESHOLD = 0.4
MAX_AGE = 2.0          # seconds — older than this and we have gone blind


class DetectionWatcher:
    """Attach to any spinning node; answers `sees(label)`."""

    def __init__(self, node: Node, topic: str = "/robot/ai/detections",
                 max_age: float = MAX_AGE):
        self.node = node
        self.max_age = max_age
        self.latest: dict[str, float] = {}
        self._stamp = None
        node.create_subscription(Detection2DArray, topic, self._on_detections, 10)

    def _on_detections(self, msg: Detection2DArray):
        seen: dict[str, float] = {}
        for det in msg.detections:
            for hyp in det.results:
                label = hyp.hypothesis.class_id
                seen[label] = max(hyp.hypothesis.score, seen.get(label, 0.0))
        self.latest = seen
        self._stamp = self.node.get_clock().now()

    def fresh(self) -> bool:
        """Is perception actually alive right now?"""
        if self._stamp is None:
            return False
        age = (self.node.get_clock().now() - self._stamp).nanoseconds / 1e9
        return age <= self.max_age

    def sees(self, label: str, min_conf: float = CONF_THRESHOLD) -> bool:
        return self.fresh() and self.latest.get(label, 0.0) >= min_conf