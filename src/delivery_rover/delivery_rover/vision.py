# FILE: src/delivery_rover/delivery_rover/vision.py
"""RobotVisionBrain — camera in, named objects out (Ch 5-6).

Grab a frame (webcam or video file), run YOLO, publish three topics:

    /robot/camera/image_raw    sensor_msgs/Image      the raw feed
    /robot/camera/annotated    sensor_msgs/Image      boxes drawn on it
    /robot/ai/detections       vision_msgs/Detection2DArray

The model loads lazily on the first frame, so constructing the node is
instant and the one-time model download doesn't stall the executor.

Inference runs in its own worker thread, never in a ROS callback: a
callback that blocks for an inference-length beat steals executor time
from everything else in the process (the sim's TF, subscriptions). The
timer callback only hands the newest frame to the worker. Rule worth
keeping: callbacks communicate, workers compute.
"""
from __future__ import annotations

import threading
import time

import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

FRAME_RATE = 5.0        # Hz — perception does not need control rates
CONF_THRESHOLD = 0.4
IMG_SIZE = 640          # inference resolution; see _process()


def to_image_msg(frame: np.ndarray, stamp, frame_id: str) -> Image:
    """OpenCV frame (H x W x 3, BGR) -> sensor_msgs/Image, by hand.

    This is everything cv_bridge does for a bgr8 image — seeing the
    message anatomy once is worth more than importing the adapter.
    """
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = "bgr8"           # mislabel this and red/blue swap (Ch 5)
    msg.step = msg.width * 3        # bytes per row
    msg.data = frame.tobytes()
    return msg


def pick_device() -> str:
    """Metal on Apple Silicon, CUDA on a Jetson/PC, else CPU."""
    import torch
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


class RobotVisionBrain(Node):
    def __init__(self, source: int | str = 0, model_name: str = "yolo11n.pt",
                 device: str | None = None, rate_hz: float = FRAME_RATE):
        super().__init__("robot_vision_brain")
        self.source = source
        self.model_name = model_name
        self.device = device or pick_device()
        self.model = None                      # loaded on first frame
        self.capture = cv2.VideoCapture(source)
        if not self.capture.isOpened():
            raise RuntimeError(
                f"could not open camera source {source!r} — on macOS check "
                "System Settings > Privacy & Security > Camera")

        qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(
            Image, "/robot/camera/image_raw", qos)
        self.annotated_pub = self.create_publisher(
            Image, "/robot/camera/annotated", qos)
        self.det_pub = self.create_publisher(
            Detection2DArray, "/robot/ai/detections", 10)
        self.latest = {}                       # label -> confidence, last frame
        self._pending = None                   # newest frame awaiting the worker
        self._lock = threading.Lock()
        self._alive = True
        self._worker = threading.Thread(target=self._work, daemon=True)
        self._worker.start()
        self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        """ROS timer: grab a frame, hand it over, return immediately."""
        ok, frame = self.capture.read()
        if not ok:
            if isinstance(self.source, str):          # video file: loop it
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        with self._lock:
            self._pending = frame                     # keep only the newest

    def _work(self):
        """Worker thread: run YOLO on the newest frame, publish, repeat."""
        while self._alive:
            with self._lock:
                frame, self._pending = self._pending, None
            if frame is None:
                time.sleep(0.02)
                continue
            self._process(frame)

    def _process(self, frame):
        if self.model is None:
            import os

            import torch
            from ultralytics import YOLO  # heavy import, done once
            if self.device == "cpu":
                # CPU inference must not starve the control loop: leave
                # cores free for MPPI. (Irrelevant on mps/cuda.)
                torch.set_num_threads(max(1, (os.cpu_count() or 2) // 2))
            self.model = YOLO(self.model_name)
            self.get_logger().info(
                f"YOLO {self.model_name} ready on {self.device}")
        stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(to_image_msg(frame, stamp, "camera_link"))

        # imgsz caps the inference resolution. A MacBook camera hands over
        # 1920x1080 frames; letting YOLO run at that size costs seconds per
        # frame and buys nothing — the detector was trained at 640.
        result = self.model.predict(
            frame, device=self.device, conf=CONF_THRESHOLD,
            imgsz=IMG_SIZE, verbose=False)[0]

        detections = Detection2DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = "camera_link"
        seen = {}
        for box in result.boxes:
            label = result.names[int(box.cls)]
            conf = float(box.conf)
            seen[label] = max(conf, seen.get(label, 0.0))
            det = Detection2D()
            det.header = detections.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            x, y, w, h = box.xywh[0].tolist()
            det.bbox.center.position.x = x
            det.bbox.center.position.y = y
            det.bbox.size_x = w
            det.bbox.size_y = h
            detections.detections.append(det)
        self.det_pub.publish(detections)
        self.latest = seen

        self.annotated_pub.publish(
            to_image_msg(result.plot(), stamp, "camera_link"))

    def destroy_node(self):
        self._alive = False
        self.capture.release()
        super().destroy_node()
