# FILE: src/delivery_rover/delivery_rover/mission.py
"""The delivery mission as a behavior tree (Ch 11).

The tree orchestrates; it never does work itself. Navigation is Nav2's
job (GoTo just watches the action), perception is the vision node's job
(LookFor just reads the latest detection message). Both of those live in
other processes — the tree only ever talks to them over the ROS graph.

Every behaviour returns RUNNING while the slow thing underneath is still
happening, so the tree stays responsive at any tick rate.
"""
from __future__ import annotations

import math
import time

import py_trees
from py_trees.common import Status

from . import world
from .detections import DetectionWatcher
from .navigator import Navigator


class GoTo(py_trees.behaviour.Behaviour):
    """Send one Nav2 goal, report RUNNING until the action finishes."""

    def __init__(self, name: str, navigator: Navigator, x: float, y: float,
                 yaw: float = 0.0):
        super().__init__(name)
        self.navigator, self.goal = navigator, (x, y, yaw)

    def initialise(self):
        self.navigator.go_to(*self.goal)

    def update(self) -> Status:
        if not self.navigator.is_done():
            self.feedback_message = (
                f"{self.navigator.distance_remaining()} m to go")
            return Status.RUNNING
        return Status.SUCCESS if self.navigator.succeeded() else Status.FAILURE

    def terminate(self, new_status):
        if new_status == Status.INVALID and not self.navigator.is_done():
            self.navigator.cancel()          # pre-empted: stop the robot


class LookFor(py_trees.behaviour.Behaviour):
    """RUNNING until perception reports `label`, FAILURE on timeout.

    Reads the newest `/robot/ai/detections` message via the watcher and
    returns immediately — perception is slower than the tick rate, and
    that is fine precisely because this never blocks on it.
    """

    def __init__(self, name: str, watcher: DetectionWatcher, label: str,
                 timeout: float = 10.0):
        super().__init__(name)
        self.watcher, self.label, self.timeout = watcher, label, timeout

    def initialise(self):
        self.deadline = time.time() + self.timeout

    def update(self) -> Status:
        if self.watcher.sees(self.label):
            self.feedback_message = f"saw a {self.label}"
            return Status.SUCCESS
        if time.time() > self.deadline:
            self.feedback_message = f"no {self.label} within {self.timeout}s"
            return Status.FAILURE
        self.feedback_message = (
            "scanning..." if self.watcher.fresh() else "no detections arriving")
        return Status.RUNNING


class Announce(py_trees.behaviour.Behaviour):
    """A one-tick log line — the mission's narration."""

    def __init__(self, message: str, logger=None):
        super().__init__(f"say '{message}'")
        self.message = message
        self._ros_logger = logger

    def update(self) -> Status:
        if self._ros_logger is not None:
            self._ros_logger.info(self.message)
        else:
            self.logger.warning(self.message)
        return Status.SUCCESS


def build_mission(navigator: Navigator, watcher: DetectionWatcher,
                  cargo: str = "bottle",
                  logger=None) -> py_trees.trees.BehaviourTree:
    """drive -> confirm cargo (with a look-around recovery) -> deliver."""
    px, py_, pyaw = world.LOCATIONS["pickup"]
    dx, dy, dyaw = world.LOCATIONS["dropoff"]

    confirm = py_trees.composites.Selector("ConfirmCargo", memory=True, children=[
        LookFor("look for cargo", watcher, cargo, timeout=10.0),
        py_trees.composites.Sequence("LookAround", memory=True, children=[
            Announce(f"no {cargo} seen — turning to look around", logger),
            GoTo("turn in place", navigator, px, py_,
                 yaw=pyaw + math.pi / 2),
            LookFor("look again", watcher, cargo, timeout=10.0),
        ]),
    ])

    root = py_trees.composites.Sequence("DeliveryMission", memory=True, children=[
        Announce("mission start", logger),
        GoTo("go to pickup", navigator, px, py_, pyaw),
        confirm,
        Announce("cargo confirmed — delivering", logger),
        GoTo("go to dropoff", navigator, dx, dy, dyaw),
        Announce("delivered!", logger),
    ])
    return py_trees.trees.BehaviourTree(root)


def tick_until_done(tree: py_trees.trees.BehaviourTree, tick_hz: float = 2.0,
                    timeout: float = 600.0, on_tick=None) -> Status:
    """Tick until the mission ends. `on_tick` receives the tree each tick."""
    tree.setup(timeout=5.0)
    deadline = time.time() + timeout
    while time.time() < deadline:
        tree.tick()
        if on_tick is not None:
            on_tick(tree)
        if tree.root.status in (Status.SUCCESS, Status.FAILURE):
            return tree.root.status
        time.sleep(1.0 / tick_hz)
    return tree.root.status
