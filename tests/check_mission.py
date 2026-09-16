# FILE: tests/check_mission.py
"""Full mission, driven the way a real operator would.

Call the service, watch the status topic. Nothing here reaches inside
the robot."""
import sys
import time

from cockpit import get_cockpit
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

fails = []


def check(name, ok, extra=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name} {extra}")
    if not ok:
        fails.append(name)


cp = get_cockpit()
status = []
# Match the publisher's transient-local durability, or a client that
# connects mid-mission silently misses the current status.
cp.node.create_subscription(
    String, "/mission/status", lambda m: status.append(m.data),
    QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

client = cp.node.create_client(Trigger, "/start_mission")
check("mission node present", client.wait_for_service(timeout_sec=90.0))

check("status topic publishing", cp.wait_until(lambda: len(status) > 0, timeout=30),
      f"→ {status[-1] if status else None}")

t0 = time.time()
future = client.call_async(Trigger.Request())
cp.wait_until(lambda: future.done(), timeout=15)
resp = future.result()
check("service accepted the request", resp is not None and resp.success,
      f"({resp.message if resp else 'no response'})")

TERMINAL = ("SUCCESS", "FAILURE")
done = cp.wait_until(lambda: status and status[-1] in TERMINAL, timeout=300)
check("mission reached a terminal state", done,
      f"in {time.time()-t0:.1f}s → {status[-1] if status else None}")

seen = []
for s in status:
    if s not in seen:
        seen.append(s)
print("\n  status trail:")
for s in seen:
    print("   ", s)

check("mission SUCCEEDED", status and status[-1] == "SUCCESS")
check("tree passed through pickup", any("pickup" in s for s in seen))
check("tree passed through dropoff", any("dropoff" in s for s in seen))

print(f"\n{len(fails)} failures" + (f": {fails}" if fails else ""))
cp.shutdown()
sys.exit(1 if fails else 0)
