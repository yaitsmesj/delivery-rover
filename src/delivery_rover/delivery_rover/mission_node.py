# FILE: src/delivery_rover/delivery_rover/mission_node.py
"""The mission, as a node: `ros2 run delivery_rover mission`.

Started by the launch system alongside everything else. It waits for
Nav2, then sits idle until someone asks for a delivery:

    ros2 service call /start_mission std_srvs/srv/Trigger

Set `auto_start:=true` to run one delivery as soon as Nav2 is ready.
"""

from __future__ import annotations

import threading
import time

import py_trees
import rclpy
from py_trees.common import Status
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .detections import DetectionWatcher
from .mission import build_mission, tick_until_done
from .navigator import Navigator


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission")
        self.declare_parameter("cargo", "bottle")
        self.declare_parameter("tick_hz", 2.0)
        self.declare_parameter("auto_start", False)
        self.cargo = self.get_parameter("cargo").value
        self.tick_hz = self.get_parameter("tick_hz").value

        self.detections = DetectionWatcher(self)
        self._requested = self.get_parameter("auto_start").value
        self._busy = False
        self.create_service(Trigger, "start_mission", self._on_start)

        # The mission's state, on a topic: watchable in Foxglove, and the
        # honest way for any client to know what the robot is doing
        # without reaching inside this process.
        #
        # Transient-local, because this is state, not a stream: a Foxglove
        # panel or a notebook that connects halfway through a delivery
        # should immediately learn the current status rather than wait for
        # it to change. Same mechanism /tf_static uses.
        self.status_pub = self.create_publisher(
            String,
            "mission/status",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._say("idle")

    def _say(self, text: str):
        self.status_pub.publish(String(data=text))

    def _on_start(self, request, response):
        if self._busy:
            response.success = False
            response.message = "a mission is already running"
        else:
            self._requested = True
            response.success = True
            response.message = f"delivering: looking for '{self.cargo}'"
        return response

    def take_request(self) -> bool:
        if self._requested and not self._busy:
            self._requested = False
            return True
        return False


def _active_leaf(tree) -> str:
    """The deepest behaviour currently RUNNING — the mission in one line."""
    running = [
        b.name
        for b in tree.root.iterate()
        if b.status == Status.RUNNING and not b.children
    ]
    return running[-1] if running else tree.root.status.name


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    # The mission node is spun in the background so its detection
    # subscription and its service keep working while the main thread
    # runs the (blocking) tick loop.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True, name="mission-spin").start()

    # BasicNavigator spins itself on demand, so it is deliberately NOT
    # added to the executor above.
    nav = Navigator(node_name="mission_navigator")
    node.get_logger().info("waiting for Nav2 to come up...")
    if not nav.wait_ready(timeout=90.0):
        node.get_logger().error(
            "Nav2 never became ready — is navigation.launch.py running?"
        )
        return
    node.get_logger().info(
        "ready. call /start_mission (std_srvs/srv/Trigger) to deliver."
    )

    try:
        while rclpy.ok():
            if node.take_request():
                node._busy = True
                node._say("running")
                tree = build_mission(
                    nav, node.detections, cargo=node.cargo, logger=node.get_logger()
                )

                def on_tick(t):
                    node.get_logger().debug(
                        py_trees.display.unicode_tree(t.root, show_status=True)
                    )
                    node._say(f"running: {_active_leaf(t)}")

                status = tick_until_done(tree, tick_hz=node.tick_hz, on_tick=on_tick)
                node.get_logger().info(f"mission finished: {status.name}")
                node._say(status.name)
                node._busy = False
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
