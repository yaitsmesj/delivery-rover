"""What happens when two things command /cmd_vel at once?

Sends a Nav2 goal, then halfway through starts a second publisher pushing
the opposite command, and watches what the robot actually does.

There is no arbitration on a topic: every message is delivered and the
subscriber acts on whichever arrived last. So the outcome is decided by
PUBLISH RATE, not by priority — whoever talks faster wins. Nav2's
controller runs at 20 Hz, so:

    --rate 40   (default)  the interferer wins; the rover is driven away
                           from its path and the goal usually fails
    --rate 10              Nav2 wins 2:1; the rover wobbles, loses time,
                           and usually still arrives

Run both. The second is the more interesting result, because "it still
worked" is exactly how this bug survives code review on a real robot.

    ros2 launch delivery_rover bringup.launch.py mission:=false
    python3 tests/experiment_two_publishers.py            # 40 Hz
    python3 tests/experiment_two_publishers.py --rate 10  # 10 Hz
"""
import argparse
import sys
import threading
import time

from geometry_msgs.msg import Twist

from cockpit import get_cockpit
from delivery_rover import world
from delivery_rover.navigator import Navigator

NAV2_RATE = 20.0        # controller_server's own publish rate, from nav2_params

ap = argparse.ArgumentParser()
ap.add_argument("--rate", type=float, default=40.0,
                help="interferer publish rate in Hz (Nav2 publishes at 20)")
ap.add_argument("--speed", type=float, default=-0.25,
                help="interferer linear.x; negative drives backwards")
ap.add_argument("--spin", type=float, default=1.0,
                help="interferer angular.z")
args = ap.parse_args(sys.argv[1:])

cp = get_cockpit()
cp.tf.wait_for("map", "base_link")

nav = Navigator(node_name="experiment_navigator")
print("Nav2 ready:", nav.wait_ready(timeout=90))

pub = cp.node.create_publisher(Twist, "/cmd_vel", 10)
rogue = Twist()
rogue.linear.x = args.speed
rogue.angular.z = args.spin

interfering = False


def interfere():
    period = 1.0 / args.rate
    while True:
        if interfering:
            pub.publish(rogue)
        time.sleep(period)


threading.Thread(target=interfere, daemon=True).start()

ratio = args.rate / NAV2_RATE
print(f"\ninterferer: {args.rate:g} Hz vs Nav2's {NAV2_RATE:g} Hz "
      f"({ratio:.1f}x) — expect the interferer to "
      f"{'WIN' if ratio > 1 else 'lose'}")
print("sending goal: home -> pickup")
nav.go_to(*world.LOCATIONS["pickup"])

t0 = time.time()
while not nav.is_done():
    t = time.time() - t0
    if 8.0 < t < 18.0 and not interfering:
        print(f"\n>>> t={t:4.1f}s  SECOND PUBLISHER ON "
              f"({args.speed:g} m/s, {args.spin:g} rad/s, {args.rate:g} Hz)\n")
        interfering = True
    elif t >= 18.0 and interfering:
        print(f"\n>>> t={t:4.1f}s  second publisher OFF\n")
        interfering = False
    print(f"  t={t:5.1f}s  pose={cp.tf.where_is()}  "
          f"remaining={nav.distance_remaining()}")
    time.sleep(2.0)

print("\nresult:", "SUCCEEDED" if nav.succeeded() else "FAILED")
print("final pose:", cp.tf.where_is())
print("\nEither way, look at the pose column while the second publisher was "
      "on:\nthat wobble is a robot being driven by two minds at once.")
nav.destroy()
cp.shutdown()
