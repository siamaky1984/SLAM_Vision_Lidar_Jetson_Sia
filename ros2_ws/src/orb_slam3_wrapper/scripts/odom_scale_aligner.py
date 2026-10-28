#!/usr/bin/env python3
# MIT License – see LICENSE in repo root
"""Estimate the metric scale of monocular ORB-SLAM3 odometry using a metric odometry source.

Monocular visual odometry has an arbitrary, drifting scale.  This node compares
displacement magnitudes between the visual odom and a metric odom (lidar ICP)
over short baselines, keeps a median-filtered scale estimate, and republishes the
visual odom with its translation multiplied by that scale.

Displacement *norms* are compared, so the two odometry frames do not need to be
aligned with each other.

Topics:
    ~/visual_odom   (sub)  nav_msgs/Odometry  – ORB-SLAM3, arbitrary scale
    ~/metric_odom   (sub)  nav_msgs/Odometry  – ICP odom, metric
    ~/odom_scaled   (pub)  nav_msgs/Odometry  – visual odom in metres
    ~/scale         (pub)  std_msgs/Float64   – current scale estimate
    ~/odom_scaled_twist (pub) nav_msgs/Odometry – metric body-frame velocity only
                                   (child_frame_id=base_link), for EKF fusion.  Derived
                                   from consecutive visual poses, so it is independent of
                                   ORB-SLAM3's arbitrary world-frame orientation.
"""

import copy
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


def _xyz(msg):
    p = msg.pose.pose.position
    return np.array([p.x, p.y, p.z])


def _quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rot_to_rotvec(R):
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_a)
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-9:  # angle ~ pi: fall back to symmetric part
        return np.zeros(3)
    return axis / n * angle


def _optical_to_base(v):
    """Camera optical (x right, y down, z fwd) -> base_link (x fwd, y left, z up)."""
    return np.array([v[2], -v[0], -v[1]])


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class OdomScaleAligner(Node):
    def __init__(self):
        super().__init__('odom_scale_aligner')
        self.declare_parameter('min_baseline_m', 0.30)      # metric motion per scale sample
        self.declare_parameter('min_visual_baseline', 1e-4)  # reject near-zero visual motion
        self.declare_parameter('max_sync_dt', 0.10)          # s, visual↔metric stamp tolerance
        self.declare_parameter('window', 15)                 # samples in median filter
        self.declare_parameter('min_samples', 3)             # publish only after this many
        self.declare_parameter('smoothing', 0.3)             # EMA factor on the median
        self.declare_parameter('visual_frame_is_optical', True)  # ORB-SLAM3 uses optical axes
        self.declare_parameter('max_speed', 3.0)             # m/s, reject VO jumps (map reset)
        self.declare_parameter('max_rate', 1.5)              # rad/s, reject VO jumps
        self.declare_parameter('twist_lin_var', 0.05)        # (m/s)^2
        self.declare_parameter('twist_ang_var', 0.10)        # (rad/s)^2

        self.min_baseline = self.get_parameter('min_baseline_m').value
        self.min_vis = self.get_parameter('min_visual_baseline').value
        self.max_dt = self.get_parameter('max_sync_dt').value
        self.min_samples = self.get_parameter('min_samples').value
        self.alpha = self.get_parameter('smoothing').value
        self.optical = self.get_parameter('visual_frame_is_optical').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_rate = self.get_parameter('max_rate').value
        self.lin_var = self.get_parameter('twist_lin_var').value
        self.ang_var = self.get_parameter('twist_ang_var').value
        self.prev_vis = None  # (stamp, xyz, R)
        self.samples = deque(maxlen=self.get_parameter('window').value)

        self.metric_buf = deque(maxlen=200)  # (stamp, xyz)
        self.anchor = None                   # (visual_xyz, metric_xyz)
        self.scale = None

        self.create_subscription(Odometry, '~/metric_odom', self._on_metric, 50)
        self.create_subscription(Odometry, '~/visual_odom', self._on_visual,
                                 qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '~/odom_scaled', 10)
        self.pub_scale = self.create_publisher(Float64, '~/scale', 10)
        self.pub_twist = self.create_publisher(Odometry, '~/odom_scaled_twist', 10)

    def _on_metric(self, msg):
        self.metric_buf.append((_stamp(msg), _xyz(msg)))

    def _metric_at(self, t):
        """Linearly interpolate the metric position at time t, or None if unavailable."""
        buf = self.metric_buf
        if len(buf) < 2 or t < buf[0][0] - self.max_dt or t > buf[-1][0] + self.max_dt:
            return None
        for (t0, p0), (t1, p1) in zip(buf, list(buf)[1:]):
            if t0 <= t <= t1:
                w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return p0 + w * (p1 - p0)
        return buf[-1][1] if t > buf[-1][0] else buf[0][1]

    def _on_visual(self, msg):
        vis = _xyz(msg)
        met = self._metric_at(_stamp(msg))
        if met is None:
            return

        if self.anchor is None:
            self.anchor = (vis, met)
        else:
            d_met = np.linalg.norm(met - self.anchor[1])
            if d_met >= self.min_baseline:
                d_vis = np.linalg.norm(vis - self.anchor[0])
                if d_vis > self.min_vis:
                    self.samples.append(d_met / d_vis)
                self.anchor = (vis, met)

        if len(self.samples) < self.min_samples:
            return

        median = float(np.median(self.samples))
        self.scale = median if self.scale is None else \
            (1.0 - self.alpha) * self.scale + self.alpha * median

        scaled = copy.deepcopy(msg)
        scaled.pose.pose.position.x *= self.scale
        scaled.pose.pose.position.y *= self.scale
        scaled.pose.pose.position.z *= self.scale
        scaled.twist.twist.linear.x *= self.scale
        scaled.twist.twist.linear.y *= self.scale
        scaled.twist.twist.linear.z *= self.scale
        self.pub_odom.publish(scaled)
        self.pub_scale.publish(Float64(data=self.scale))
        self._publish_twist(msg, vis)

    def _publish_twist(self, msg, vis):
        t = _stamp(msg)
        R = _quat_to_rot(msg.pose.pose.orientation)
        prev, self.prev_vis = self.prev_vis, (t, vis, R)
        if prev is None:
            return
        dt = t - prev[0]
        if dt <= 1e-3 or dt > 1.0:
            return

        v_body = prev[2].T @ (vis - prev[1]) * self.scale / dt
        w_body = _rot_to_rotvec(prev[2].T @ R) / dt
        if self.optical:
            v_body, w_body = _optical_to_base(v_body), _optical_to_base(w_body)

        # Tracking re-initialisation shows up as an impossible jump: drop it.
        if np.linalg.norm(v_body) > self.max_speed or abs(w_body[2]) > self.max_rate:
            self.get_logger().warn('Visual odom jump rejected (tracking reset?)',
                                   throttle_duration_sec=2.0)
            return

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.orientation.w = 1.0
        out.twist.twist.linear.x = float(v_body[0])
        out.twist.twist.linear.y = float(v_body[1])
        out.twist.twist.angular.z = float(w_body[2])
        cov = [0.0] * 36
        for i in (0, 7):
            cov[i] = self.lin_var
        cov[14] = 1e3
        cov[21] = cov[28] = 1e3
        cov[35] = self.ang_var
        out.twist.covariance = cov
        out.pose.covariance = [1e3 if i % 7 == 0 else 0.0 for i in range(36)]
        self.pub_twist.publish(out)


def main():
    rclpy.init()
    node = OdomScaleAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
