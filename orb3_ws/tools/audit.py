#!/usr/bin/env python3
"""Measure live sensor delivery and ORB camera-pose output over a bounded interval."""
import argparse
import bisect
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, Int32


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=20.0)
    p.add_argument('--output', required=True)
    p.add_argument('--best-effort-images', action='store_true')
    a = p.parse_args()
    rclpy.init()
    node = rclpy.create_node('orb3_runtime_audit')
    stamps = {k: [] for k in ['image', 'depth', 'imu', 'pose']}
    xyz, valid = [], []
    states, initialized = [], []
    subscriptions = []

    def receive(key, msg):
        stamps[key].append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if key == 'pose':
            pos = msg.pose.position
            xyz.append([pos.x, pos.y, pos.z])
        if key == 'depth' and len(valid) < 20:
            dtype = '<u2' if msg.encoding == '16UC1' else '<f4'
            itemsize = np.dtype(dtype).itemsize
            image = np.ndarray((msg.height, msg.width), dtype=dtype,
                               buffer=msg.data, strides=(msg.step, itemsize))
            valid.append(float(np.mean(np.isfinite(image) & (image > 0))))

    for key, typ, topic in [
        ('image', Image, '/camera/camera/infra1/image_rect_raw'),
        ('depth', Image, '/camera/camera/depth/image_rect_raw'),
        ('imu', Imu, '/camera/camera/imu'),
        ('pose', PoseStamped, '/orb3/pose')]:
        qos = qos_profile_sensor_data
        if key in ('image', 'depth') and not a.best_effort_images:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        subscriptions.append(node.create_subscription(
            typ, topic, lambda msg, k=key: receive(k, msg), qos))
    subscriptions.append(node.create_subscription(Int32, '/orb3/tracking_state',
        lambda m: states.append(m.data), 10))
    subscriptions.append(node.create_subscription(Bool, '/orb3/imu_initialized',
        lambda m: initialized.append(m.data), 10))
    deadline = time.monotonic() + a.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = {'duration_seconds': a.seconds, 'streams': {}}
    for key, times in stamps.items():
        delta = np.diff(times)
        result['streams'][key] = {
            'count': len(times),
            'mean_hz': float((len(times)-1)/(times[-1]-times[0])) if len(times) > 1 and times[-1] > times[0] else None,
            'strictly_monotonic': bool(np.all(delta > 0)),
            'max_gap_seconds': float(np.max(delta)) if len(delta) else None}
    nearest = []
    depth = sorted(stamps['depth'])
    for t in stamps['image']:
        i = bisect.bisect_left(depth, t)
        candidates = depth[max(0, i-1):i+1]
        if candidates:
            nearest.append(min(abs(t-s) for s in candidates))
    result['nearest_depth_image_dt_median_seconds'] = float(np.median(nearest)) if nearest else None
    result['nonzero_finite_depth_fraction_median'] = float(np.median(valid)) if valid else None
    positions = np.asarray(xyz)
    result['pose_finite'] = bool(np.all(np.isfinite(positions))) if xyz else None
    result['pose_axis_span_metres'] = np.ptp(positions, axis=0).tolist() if xyz else None
    result['tracking_state_counts'] = {str(s): states.count(s) for s in set(states)}
    result['imu_initialized_seen'] = any(initialized)
    Path(a.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
