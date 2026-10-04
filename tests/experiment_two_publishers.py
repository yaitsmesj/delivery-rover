"""What happens when two things command /cmd_vel at once?

Sends a Nav2 goal, then halfway through starts publishing a competing
Twist, and watches what the robot actually does. Run it with the stack
already up:

    ros2 launch delivery_rover bringup.launch.py mission:=false
    python3 tests/experiment_two_publishers.py
"""
import threading
import time

from geometry_msgs.msg import Twist

from cockpit import get_cockpit
from delivery_rover import world
from delivery_rover.navigator import Navigator

cp = get_cockpit()
cp.tf.wait_for("map", "base_link")

nav = Navigator(node_name="experiment_navigator")
print("Nav2 ready:", nav.wait_ready(timeout=90))

pub = cp.node.create_publisher(Twist, "/cmd_vel", 10)
rogue = Twist()
rogue.linear.x = -0.25          # backwards, while Nav2 wants to go forwards
rogue.angular.z = 1.0           # and spinning

interfering = False


def interfere():
    while True:
        if interfering:
            pub.publish(rogue)
        time.sleep(0.1)          # 10 Hz, same order as Nav2's 20 Hz


threading.Thread(target=interfere, daemon=True).start()

print("\nsending goal: home -> pickup")
nav.go_to(*world.LOCATIONS["pickup"])

t0 = time.time()
while not nav.is_done():
    t = time.time() - t0
    if 8.0 < t < 18.0 and not interfering:
        print(f"\n>>> t={t:4.1f}s  SECOND PUBLISHER ON (reverse + spin)\n")
        interfering = True
    elif t >= 18.0 and interfering:
        print(f"\n>>> t={t:4.1f}s  second publisher OFF\n")
        interfering = False
    print(f"  t={t:5.1f}s  pose={cp.tf.where_is()}  "
          f"remaining={nav.distance_remaining()}")
    time.sleep(2.0)

print("\nresult:", "SUCCEEDED" if nav.succeeded() else "FAILED")
print("final pose:", cp.tf.where_is())
nav.destroy()
cp.shutdown()
