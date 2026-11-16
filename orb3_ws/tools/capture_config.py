#!/usr/bin/env python3
"""Capture current rectified IR/depth geometry and the exact IMU message frame."""
import argparse
import json
import time
from pathlib import Path

import numpy as np


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = x*x + y*y + z*z + w*w
    if abs(norm - 1.0) > 1e-3:
        raise ValueError('TF quaternion is not normalized')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix(transform):
    result = np.eye(4)
    result[:3, :3] = rotation(transform.rotation)
    t = transform.translation
    result[:3, 3] = [t.x, t.y, t.z]
    return result


def validate_intrinsics_profile(profile, width, height, driver_projection, serial):
    if profile['device_serial'] != serial:
        raise ValueError('Calibration belongs to a different SDK camera serial')
    if profile['image_topic'] != '/camera/camera/infra1/image_rect_raw':
        raise ValueError('Calibration belongs to a different image stream')
    if profile['image_size'] != [width, height]:
        raise ValueError('Calibration resolution differs from the active image stream')
    if not np.allclose(profile['reference_driver_projection'], driver_projection, atol=1e-4):
        raise ValueError('Driver projection changed; review crop/rectification before reusing calibration')
    K = np.asarray(profile['K'], dtype=float)
    D = np.asarray(profile['D'], dtype=float)
    if K.shape != (3, 3) or D.shape != (5,) or not np.isfinite(K).all() or not np.isfinite(D).all():
        raise ValueError('Invalid calibration matrix or distortion coefficients')
    if K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1]):
        raise ValueError('Invalid pinhole intrinsic matrix')
    projection = [K[0, 0], 0, K[0, 2], 0, 0, K[1, 1], K[1, 2], 0, 0, 0, 1, 0]
    return projection, D.tolist()


def write_settings(path, projection, width, height, Tbc, hz, factor, distortion=None, baseline=0.05):
    p = projection
    d = [0.0] * 5 if distortion is None else distortion
    values = ', '.join(f'{v:.12f}' for v in Tbc.ravel())
    text = f'''%YAML:1.0
# Auto-captured camera geometry. IMU noise below is an initial tuning value,
# not a measured noise calibration. Translation is in metres.
File.version: "1.0"
Camera.type: "PinHole"
Camera1.fx: {float(p[0]):.12f}
Camera1.fy: {float(p[5]):.12f}
Camera1.cx: {float(p[2]):.12f}
Camera1.cy: {float(p[6]):.12f}
Camera1.k1: {float(d[0]):.15f}
Camera1.k2: {float(d[1]):.15f}
Camera1.p1: {float(d[2]):.15f}
Camera1.p2: {float(d[3]):.15f}
Camera1.k3: {float(d[4]):.15f}
Camera.width: {width}
Camera.height: {height}
Camera.fps: 30
Camera.RGB: 0
# Used for RGB-D close/far landmark classification, not depth unit conversion.
Stereo.b: {float(baseline):.12f}
Stereo.ThDepth: 40.0
RGBD.DepthMapFactor: {float(factor):.12f}
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{values}]
IMU.InsertKFsWhenLost: 0
IMU.NoiseGyro: 0.01
IMU.NoiseAcc: 0.1
IMU.GyroWalk: 0.000001
IMU.AccWalk: 0.0001
IMU.Frequency: {float(hz):.6f}
ORBextractor.nFeatures: 1250
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
'''
    Path(path).write_text(text)


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from realsense2_camera_msgs.srv import DeviceInfo
    from tf2_ros import Buffer, TransformListener
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=25.0)
    parser.add_argument('--intrinsics-profile', type=Path)
    args = parser.parse_args()
    profile = json.loads(args.intrinsics_profile.read_text()) if args.intrinsics_profile else None
    rclpy.init()
    node = rclpy.create_node('orb3_capture_geometry')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    messages, stamps = {}, []
    subscriptions = []
    client = node.create_client(DeviceInfo, '/camera/camera/device_info')
    device_future = client.call_async(DeviceInfo.Request()) if client.wait_for_service(timeout_sec=2.0) else None
    if profile and device_future is None:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Camera device_info service is required to verify the calibration serial')

    def receive(key, msg):
        messages[key] = msg
        if key == 'imu':
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamps and t <= stamps[-1]:
                raise ValueError('IMU timestamps are not strictly increasing')
            stamps.append(t)

    base = '/camera/camera/'
    for key, typ, topic in [
        ('ir', CameraInfo, 'infra1/camera_info'),
        ('depth', CameraInfo, 'depth/camera_info'),
        ('depth_image', Image, 'depth/image_rect_raw'),
        ('ir_image', Image, 'infra1/image_rect_raw'),
        ('imu', Imu, 'imu'),
    ]:
        subscriptions.append(node.create_subscription(
            typ, base+topic, lambda m, k=key: receive(k, m), qos_profile_sensor_data))
    deadline = time.monotonic() + args.timeout
    transforms = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(messages) != 5 or len(stamps) < 100:
                continue
            if profile and not device_future.done():
                continue
            ir, depth, imu = messages['ir'], messages['depth'], messages['imu']
            try:
                transforms = [
                    buffer.lookup_transform(imu.header.frame_id, ir.header.frame_id, Time()),
                    buffer.lookup_transform(ir.header.frame_id, depth.header.frame_id, Time())]
                break
            except Exception:
                continue
        if transforms is None:
            raise RuntimeError('未收到完整相机/IMU/TF 数据；请先启动相机并检查 ROS_DOMAIN_ID。已收到：'+str(list(messages)))
        ir, depth = messages['ir'], messages['depth']
        if (ir.width, ir.height) != (depth.width, depth.height):
            raise ValueError('IR/depth resolutions differ')
        if not np.allclose(ir.p, depth.p, atol=1e-4):
            raise ValueError('IR/depth projection matrices differ; depth needs reprojection')
        if not np.allclose(matrix(transforms[1].transform), np.eye(4), atol=1e-4):
            raise ValueError('IR/depth optical frames differ; depth needs reprojection')
        if not np.allclose(np.reshape(ir.r, (3, 3)), np.eye(3), atol=1e-5):
            raise ValueError('Non-identity rectification requires an adjusted camera-IMU transform')
        ir_image = messages['ir_image']
        if ir_image.encoding != 'mono8':
            raise ValueError('Expected mono8 infrared images, received '+ir_image.encoding)
        for key, info in [('ir_image', ir), ('depth_image', depth)]:
            image = messages[key]
            if (image.width, image.height, image.header.frame_id) != (info.width, info.height, info.header.frame_id):
                raise ValueError('Image/CameraInfo geometry mismatch: '+key)
        encoding = messages['depth_image'].encoding
        # The RealSense ROS driver publishes 16UC1 in millimetres.
        if encoding not in ('16UC1', '32FC1'):
            raise ValueError('Unsupported depth encoding: '+encoding)
        factor = 1000.0 if encoding == '16UC1' else 1.0
        hz = 1.0 / float(np.median(np.diff(stamps)))
        if not 50 < hz < 1000:
            raise ValueError('Unexpected IMU frequency: '+str(hz))
        Tbc = matrix(transforms[0].transform)
        device_serial = device_future.result().serial_number if device_future and device_future.done() else None
        projection, distortion, baseline = list(ir.p), [0.0] * 5, 0.05
        if profile:
            projection, distortion = validate_intrinsics_profile(
                profile, ir.width, ir.height, ir.p, device_serial)
            baseline = float(profile['baseline_m'])
        write_settings(args.output, projection, ir.width, ir.height, Tbc, hz, factor, distortion, baseline)
        metadata = {'captured_at_unix': time.time(), 'ir_frame': ir.header.frame_id,
                    'depth_frame': depth.header.frame_id, 'imu_frame': messages['imu'].header.frame_id,
                    'device_serial': device_serial,
                    'driver_projection': list(ir.p), 'projection': [float(x) for x in projection],
                    'distortion': distortion, 'size': [ir.width, ir.height],
                    'T_b_c1': Tbc.tolist(), 'imu_hz': hz, 'depth_encoding': encoding,
                    'depth_map_factor': factor, 'imu_noise_calibrated': False,
                    'source': 'live ROS geometry/TF + VINS checkerboard intrinsics' if profile else 'live ROS CameraInfo and TF',
                    'intrinsics_profile': str(args.intrinsics_profile) if profile else None,
                    'camera_imu_calibrated': False}
        Path(args.output+'.json').write_text(json.dumps(metadata, indent=2)+'\n')
        print(f'配置已采集：{args.output}；IMU {hz:.1f} Hz，深度 {encoding}')
        if profile:
            print('内参来源：'+profile['name']+'；SDK 设备序列号已核对：'+device_serial)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
