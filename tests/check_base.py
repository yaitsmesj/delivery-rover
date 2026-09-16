# FILE: tests/check_base.py
"""M1 gate, automated. Run it with the robot already launched.

This does by script exactly what the M1 section does by hand with
`tf2_echo`, `ros2 topic hz` and `ros2 topic echo`. It is a tool for you,
not part of the robot — which is why it lives in tests/ and not in the
package.

    python3 tests/check_base.py
"""
import sys

from cockpit import get_cockpit
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

fails = []


def check(name, ok, extra=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name} {extra}")
    if not ok:
        fails.append(name)


cp = get_cockpit()

check("TF chain map->base_link discovered", cp.tf.wait_for("map", "base_link"))
pose = None
try:
    pose = cp.tf.where_is("map", "base_link")
    print("      pose:", pose)
except Exception as e:                                   # noqa: BLE001
    check("where_is()", False, f"({e})")

scans = []
cp.node.create_subscription(
    LaserScan, "/scan", scans.append,
    QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT))
check("/scan arriving", cp.wait_until(lambda: len(scans) > 0))

if scans:
    scan = scans[-1]
    check("scan has 240 rays", len(scan.ranges) == 240, f"({len(scan.ranges)})")
    check("scan frame is laser_link", scan.header.frame_id == "laser_link",
          f"({scan.header.frame_id})")
    check("range_max is 8 m", abs(scan.range_max - 8.0) < 1e-6)

odoms = []
cp.node.create_subscription(Odometry, "/odom", odoms.append, 10)
check("/odom arriving", cp.wait_until(lambda: len(odoms) > 0))
if odoms:
    o = odoms[-1]
    check("odom frame_id is odom", o.header.frame_id == "odom",
          f"({o.header.frame_id})")
    check("odom child is base_link", o.child_frame_id == "base_link",
          f"({o.child_frame_id})")

# The derivable check: from home, the ray dead ahead must hit shelf 2 at
# x = 6.0. Only meaningful if the rover has not been driven away from home.
if scans and pose and abs(pose[0] - 1.0) < 0.05 and abs(pose[1] - 1.0) < 0.05:
    ahead = scans[-1].ranges[len(scans[-1].ranges) // 2]
    check("straight-ahead ray = 5.0 m", abs(ahead - 5.0) < 0.1, f"({ahead:.2f})")
else:
    print("SKIP  straight-ahead ray — rover is not at home")

print(f"\n{len(fails)} failures" + (f": {fails}" if fails else ""))
cp.shutdown()
sys.exit(1 if fails else 0)
