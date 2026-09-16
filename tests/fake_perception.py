# FILE: tests/fake_perception.py
"""Stand-in for the vision node: publish detections without running YOLO.

Lets the mission tree be tested for its own logic, at a fraction of the
CPU. `--label ""` publishes empty detections, which is how the recovery
branch and the clean-failure path get exercised deterministically.
"""
import argparse
import sys

import rclpy
import rclpy.executors
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class FakePerception(Node):
    def __init__(self, label: str, score: float, rate: float,
                 delay: float = 0.0):
        super().__init__("robot_vision_brain")
        self.label, self.score = label, score
        # Simulates "the bottle is revealed N seconds in", which is how the
        # recovery branch gets exercised deterministically.
        self.delay = delay
        self.start = self.get_clock().now()
        self.pub = self.create_publisher(Detection2DArray,
                                         "/robot/ai/detections", 10)
        self.create_timer(1.0 / rate, self._tick)

        # This node publishes and never prints on its own, which makes a
        # working run and a hung one look identical. So say what it is
        # doing, once at startup and then on a heartbeat.
        what = f"'{label}' at score {score}" if label else "NOTHING (empty detections)"
        after = f" after a {delay:.0f}s delay" if delay else ""
        self.get_logger().info(
            f"publishing {what}{after} on /robot/ai/detections at {rate:g} Hz. "
            "Ctrl-C to stop.")

    def _tick(self):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        age = (self.get_clock().now() - self.start).nanoseconds / 1e9
        if self.label and age >= self.delay:
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.label
            hyp.hypothesis.score = self.score
            det.results.append(hyp)
            det.bbox.size_x, det.bbox.size_y = 60.0, 80.0
            msg.detections.append(det)
        self.pub.publish(msg)
        self.get_logger().info(
            f"t={age:5.1f}s  publishing {len(msg.detections)} detection(s)"
            + (f" — waiting for the {self.delay:.0f}s delay"
               if self.label and age < self.delay else ""),
            throttle_duration_sec=2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="bottle")
    ap.add_argument("--score", type=float, default=0.9)
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--delay", type=float, default=0.0)
    args = ap.parse_args(sys.argv[1:])
    rclpy.init()
    node = FakePerception(args.label, args.score, args.rate, args.delay)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass                              # Ctrl-C is the normal way to stop
    finally:
        print("stopped.")               # not the ROS logger: by now the
        node.destroy_node()             # context may already be torn down
        if rclpy.ok():
            rclpy.shutdown()


main()
