"""Client-side check: the stack is already launched; drive it.

Run with the workspace sourced, from any directory:
    python3 tests/test_nav_client.py
"""
import sys
import time

from cockpit import get_cockpit
from delivery_rover.navigator import Navigator
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from delivery_rover import world

fails = []


def check(name, ok, extra=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name} {extra}")
    if not ok:
        fails.append(name)


cp = get_cockpit()

# --- the M1 contract, verified from a pure client -----------------------
check("TF chain map->base_link discovered", cp.tf.wait_for("map", "base_link"))
try:
    print("      pose:", cp.tf.where_is())
except Exception as e:
    check("where_is()", False, f"({e})")

scans = []
cp.node.create_subscription(
    LaserScan, "/scan", scans.append,
    QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT))
check("/scan arriving", cp.wait_until(lambda: len(scans) > 0),
      f"({len(scans)} msgs)")
if scans:
    n = len(scans[-1].ranges)
    check("scan has 240 rays", n == 240, f"({n})")

odoms = []
cp.node.create_subscription(Odometry, "/odom", odoms.append, 10)
check("/odom arriving", cp.wait_until(lambda: len(odoms) > 0),
      f"({len(odoms)} msgs)")

# --- Nav2 ---------------------------------------------------------------
nav = Navigator(node_name="cockpit_navigator")
t0 = time.time()
ready = nav.wait_ready(timeout=90.0)
check("Nav2 ready (lifecycle + costmaps)", ready, f"in {time.time()-t0:.1f}s")

if ready:
    t0 = time.time()
    ok = nav.go_to_blocking(*world.LOCATIONS["pickup"], timeout=120)
    check("home -> pickup", ok, f"in {time.time()-t0:.1f}s")
    print("      pose:", cp.tf.where_is())

    t0 = time.time()
    ok = nav.go_to_blocking(*world.LOCATIONS["dropoff"], timeout=150)
    check("pickup -> dropoff", ok, f"in {time.time()-t0:.1f}s")
    print("      pose:", cp.tf.where_is())

print(f"\n{len(fails)} failures" + (f": {fails}" if fails else ""))
nav.destroy()
cp.shutdown()
sys.exit(1 if fails else 0)
