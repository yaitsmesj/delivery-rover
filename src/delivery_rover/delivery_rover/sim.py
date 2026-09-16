# FILE: src/delivery_rover/delivery_rover/sim.py
"""DiffDriveSim — the rover base, simulated.

A kinematic differential-drive robot: subscribe /cmd_vel, integrate the
unicycle model, publish /odom + TF, and ray-cast a lidar against the depot
walls. No physics engine — the 20% of simulation that gives 80% of the
value for navigation work.

Frames (REP-105): map -> odom -> base_link, plus static base_link ->
laser_link and base_link -> camera_link. In this sim odometry is perfect,
so map -> odom is identity and there is no localizer to run.
"""
from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from . import world

SIM_RATE = 50.0        # Hz — integration
ODOM_RATE = 20.0       # Hz — /odom + TF publish
SCAN_RATE = 10.0       # Hz — /scan publish
CMD_TIMEOUT = 0.5      # s — stop if the controller goes quiet
SCAN_RAYS = 240
SCAN_MAX = 8.0         # m


def yaw_to_quat(yaw: float):
    """Planar yaw -> quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DiffDriveSim(Node):
    def __init__(self, start=world.LOCATIONS["home"]):
        super().__init__("diff_drive_sim")
        self.x, self.y, self.yaw = start
        self.v, self.w = 0.0, 0.0
        self._last_cmd_time = self.get_clock().now()
        self._grid = world.occupancy_grid()

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        sensor_qos = QoSProfile(
            depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", sensor_qos)

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_static_frames()

        self.create_timer(1.0 / SIM_RATE, self._integrate)
        self.create_timer(1.0 / ODOM_RATE, self._publish_odom)
        self.create_timer(1.0 / SCAN_RATE, self._publish_scan)

    # -- motion ------------------------------------------------------------
    def _on_cmd(self, msg: Twist):
        self.v, self.w = msg.linear.x, msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    def _integrate(self):
        dt = 1.0 / SIM_RATE
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age > CMD_TIMEOUT:            # dead-man switch, like a real base
            self.v, self.w = 0.0, 0.0
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        if world.is_occupied(nx, ny):    # walls are real: refuse to enter
            self.v = 0.0
        else:
            self.x, self.y = nx, ny
        self.yaw = math.atan2(math.sin(self.yaw + self.w * dt),
                              math.cos(self.yaw + self.w * dt))

    # -- outputs -----------------------------------------------------------
    def _publish_odom(self):
        now = self.get_clock().now().to_msg()
        _qx, _qy, qz, qw = yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

    def _publish_scan(self):
        angles = np.linspace(-math.pi, math.pi, SCAN_RAYS, endpoint=False)
        ranges = self._raycast(angles + self.yaw)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_link"
        scan.angle_min, scan.angle_max = float(angles[0]), float(angles[-1])
        scan.angle_increment = float(angles[1] - angles[0])
        scan.range_min, scan.range_max = 0.05, SCAN_MAX
        scan.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(scan)

    def _raycast(self, angles: np.ndarray) -> np.ndarray:
        """March every ray through the occupancy grid until it hits a wall."""
        step = world.RESOLUTION / 2.0
        n_steps = int(SCAN_MAX / step)
        ds = (np.arange(1, n_steps + 1) * step)[:, None]        # (steps, 1)
        xs = self.x + ds * np.cos(angles)[None, :]              # (steps, rays)
        ys = self.y + ds * np.sin(angles)[None, :]
        cols = (xs / world.RESOLUTION).astype(int)
        rows = (ys / world.RESOLUTION).astype(int)
        inside = ((0 <= cols) & (cols < self._grid.shape[1]) &
                  (0 <= rows) & (rows < self._grid.shape[0]))
        hit = np.where(inside, self._grid[rows.clip(0, self._grid.shape[0] - 1),
                                          cols.clip(0, self._grid.shape[1] - 1)], 1)
        first = np.argmax(hit == 1, axis=0)                     # first wall
        no_hit = ~hit.any(axis=0)
        dist = (first + 1) * step
        dist[no_hit] = SCAN_MAX
        return dist

    def _publish_static_frames(self):
        frames = []
        for child, (dx, dz) in {"laser_link": (0.0, 0.15),
                                "camera_link": (0.10, 0.20)}.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = child
            t.transform.translation.x = dx
            t.transform.translation.z = dz
            t.transform.rotation.w = 1.0
            frames.append(t)
        # Perfect odometry: map -> odom is identity, published once.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.rotation.w = 1.0
        frames.append(t)
        self.static_tf.sendTransform(frames)

    # -- notebook conveniences --------------------------------------------
    def teleport(self, x: float, y: float, yaw: float = 0.0):
        self.x, self.y, self.yaw = x, y, yaw

    @property
    def pose(self):
        return (round(self.x, 3), round(self.y, 3), round(self.yaw, 3))
