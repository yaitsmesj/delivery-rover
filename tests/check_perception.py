"""Client-side check of perception: subscribe like the mission does."""
import sys

from cockpit import get_cockpit
from delivery_rover.detections import DetectionWatcher
from sensor_msgs.msg import Image

fails = []


def check(name, ok, extra=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name} {extra}")
    if not ok:
        fails.append(name)


cp = get_cockpit()
watch = DetectionWatcher(cp.node)

raw, annotated = [], []
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
cp.node.create_subscription(Image, "/robot/camera/image_raw", raw.append, qos)
cp.node.create_subscription(Image, "/robot/camera/annotated", annotated.append, qos)

check("detections arriving", cp.wait_until(lambda: watch.latest != {}, timeout=30),
      f"→ {watch.latest}")
check("/robot/camera/image_raw", cp.wait_until(lambda: len(raw) > 0, timeout=15),
      f"({len(raw)} msgs)")
check("/robot/camera/annotated", cp.wait_until(lambda: len(annotated) > 0, timeout=15),
      f"({len(annotated)} msgs)")
if raw:
    check("image is bgr8", raw[-1].encoding == "bgr8", f"({raw[-1].encoding})")
    ok = raw[-1].step == raw[-1].width * 3
    check("row stride consistent", ok)

check("watcher reports fresh", watch.fresh())
labels = list(watch.latest)
check("sees() agrees with latest", all(watch.sees(l) for l in labels), f"{labels}")
check("sees() false for absent label", not watch.sees("zebra"))

print(f"\n{len(fails)} failures" + (f": {fails}" if fails else ""))
cp.shutdown()
sys.exit(1 if fails else 0)
