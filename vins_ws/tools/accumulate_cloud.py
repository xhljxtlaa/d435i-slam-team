#!/usr/bin/env python3
"""Bounded voxel map of VINS world-frame sparse points; no pose transform needed."""
import math
from collections import OrderedDict, deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry


class CloudMap(Node):
    def __init__(self):
        super().__init__('vins_cloud_accumulator')
        self.voxel = float(self.declare_parameter('voxel_size', 0.05).value)
        self.limit = int(self.declare_parameter('max_voxels', 200000).value)
        self.min_range = float(self.declare_parameter('min_range', 0.18).value)
        self.max_range = float(self.declare_parameter('max_range', 8.0).value)
        self.min_observations = int(self.declare_parameter('min_observations', 3).value)
        self.clear_on_rviz_start = bool(self.declare_parameter('clear_on_rviz_start', True).value)
        self.seen_rviz_publishers = set()
        self.frame = str(self.declare_parameter('frame', 'world').value)
        self.output = Path(str(self.declare_parameter(
            'save_path', str(Path(__file__).resolve().parents[1] / 'output/accumulated_cloud.ply')).value))
        if (self.voxel <= 0 or self.limit <= 0 or self.min_observations < 1
                or not 0 <= self.min_range < self.max_range):
            raise ValueError('Invalid voxel, capacity, observation count or range parameters')
        self.points = OrderedDict()
        self.pending = OrderedDict()
        self.poses = deque(maxlen=400)
        self.rejected = 0
        self.last_stamp = None
        self.dirty = True
        self.pub = self.create_publisher(PointCloud2, '/accumulated_cloud',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointCloud, '/point_cloud', self.receive, 10)
        self.create_subscription(Odometry, '/odometry', self.odometry, 100)
        self.create_subscription(Bool, '/vins_restart', self.restart, 10)
        self.create_subscription(Bool, '/vins_estimator_reset', self.restart, 10)
        self.create_service(Trigger, '~/clear', self.clear)
        self.create_service(Trigger, '~/save', self.save)
        self.create_service(Trigger, '~/stats', self.stats)
        self.create_timer(1.0, self.publish)
        if self.clear_on_rviz_start:
            self.create_timer(0.5, self.check_rviz_start)
        self.get_logger().info(f'Accumulating {self.frame} points, voxel={self.voxel} m, '
            f'range={self.min_range}..{self.max_range} m, confirmations={self.min_observations}, max={self.limit}')

    def check_rviz_start(self):
        # A new DDS publisher GID distinguishes a restarted RViz even if its
        # node name is unchanged and discovery never observes an empty graph.
        current = {bytes(p.endpoint_gid) for p in self.get_publishers_info_by_topic('/rosout')
                   if p.node_name in ('rviz', 'rviz2')}
        if current - self.seen_rviz_publishers:
            self.seen_rviz_publishers.update(current)
            self.clear(None, Trigger.Response())
            self.get_logger().info('New RViz process detected: accumulated map cleared')

    def odometry(self, msg):
        if msg.header.frame_id != self.frame:
            return
        p = msg.pose.pose.position
        xyz = (p.x, p.y, p.z)
        if not all(math.isfinite(v) for v in xyz):
            return
        stamp = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        self.poses.append((stamp, xyz))

    def receive(self, msg):
        if msg.header.frame_id != self.frame:
            self.get_logger().warning('Ignoring cloud outside configured world frame', throttle_duration_sec=10)
            return
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.clear(None, Trigger.Response())
            return
        if not self.poses:
            return
        pose_stamp, origin = min(self.poses, key=lambda p: abs(p[0]-stamp))
        if abs(pose_stamp-stamp) > 150000000:
            return
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        seen = set()
        for p in msg.points:
            xyz = (p.x, p.y, p.z)
            if not all(math.isfinite(v) for v in xyz):
                continue
            distance = math.dist(xyz, origin)
            if not self.min_range <= distance <= self.max_range:
                self.rejected += 1
                continue
            key = tuple(math.floor(v / self.voxel) for v in xyz)
            if key in seen:
                continue
            seen.add(key)
            previous = self.pending.get(key)
            count = previous[0]+1 if previous and stamp-previous[1] <= 1000000000 else 1
            self.pending[key] = (count, stamp)
            self.pending.move_to_end(key)
            if len(self.pending) > self.limit*2:
                self.pending.popitem(last=False)
            if count < self.min_observations:
                continue
            self.points[key] = xyz
            self.points.move_to_end(key)
            if len(self.points) > self.limit:
                self.points.popitem(last=False)
        self.dirty = True

    def publish(self):
        if not self.dirty:
            return
        header = Header(frame_id=self.frame, stamp=self.get_clock().now().to_msg())
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
        self.pub.publish(point_cloud2.create_cloud(header, fields, list(self.points.values())))
        self.dirty = False

    def clear(self, request, response):
        self.points.clear()
        self.pending.clear()
        self.poses.clear()
        self.rejected = 0
        self.last_stamp = None
        self.dirty = True
        self.publish()
        response.success = True
        response.message = 'Accumulated cloud cleared'
        return response

    def stats(self, request, response):
        response.success = True
        response.message = (f'map_points={len(self.points)}, candidate_voxels={len(self.pending)}, '
            f'rejected_by_range={self.rejected}, range={self.min_range}..{self.max_range} m, '
            f'confirmations={self.min_observations}')
        return response

    def restart(self, msg):
        if msg.data:
            self.clear(None, Trigger.Response())

    def save(self, request, response):
        try:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.output.with_suffix('.ply.tmp')
            with tmp.open('w') as f:
                f.write(f'ply\nformat ascii 1.0\nelement vertex {len(self.points)}\n'
                        'property float x\nproperty float y\nproperty float z\nend_header\n')
                for p in self.points.values():
                    f.write(' '.join(map(str, p)) + '\n')
            tmp.replace(self.output)
            response.success = True
            response.message = f'Saved {len(self.points)} points to {self.output}'
        except OSError as e:
            response.success = False
            response.message = str(e)
        return response


if __name__ == '__main__':
    rclpy.init()
    node = CloudMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
