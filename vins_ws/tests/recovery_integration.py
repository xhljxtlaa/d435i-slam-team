#!/usr/bin/env python3
"""Run only in an isolated ROS_DOMAIN_ID; inject tracking loss and verify resume."""
import os
import signal
import shutil
import subprocess
import tempfile
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Bool

WS = Path(__file__).resolve().parents[1]

if os.environ.get('ROS_DOMAIN_ID') != '73':
    raise RuntimeError('This synthetic test requires isolated ROS_DOMAIN_ID=73')
rclpy.init()
node=rclpy.create_node('recovery_integration_test')
left=node.create_publisher(Image,'/camera/camera/infra1/image_rect_raw',10)
depth=node.create_publisher(Image,'/camera/camera/depth/image_rect_raw',10)
imu=node.create_publisher(Imu,'/camera/camera/imu',10)
events=[];paths=[]
node.create_subscription(Bool,'/vins_estimator_reset',lambda m:events.append((time.monotonic(),m.data)),10)
node.create_subscription(RosPath,'/path',lambda m:paths.append((time.monotonic(),len(m.poses))),10)
texture=cv2.GaussianBlur(np.random.default_rng(9).integers(0,256,(480,640),dtype=np.uint8),(5,5),1)
blank=np.full((480,640),127,np.uint8)
measured=np.full((480,640),2000,np.uint16)
with tempfile.TemporaryDirectory(prefix='vins_recovery_') as folder:
    config=(WS / 'config/d435i/depth_imu.yaml').read_text()
    for name in ("left.yaml","right.yaml"):
        shutil.copy2((WS / 'config/d435i')/name,Path(folder)/name)
    config=config.replace('/home/he/vins_ws/output/',folder+'/')
    cfg=Path(folder)/'config.yaml';cfg.write_text(config)
    log=(Path(folder)/'recovery_integration.log').open('w')
    process=subprocess.Popen([str(WS / 'install/vins/lib/vins/vins_node'),str(cfg)],stdout=log,stderr=log)
    started=time.monotonic();last_frame=0.;last_imu=0.
    try:
        while time.monotonic()-started<16:
            if process.poll() is not None:
                raise RuntimeError(f"Isolated estimator exited early: {process.returncode}")
            now=time.monotonic();elapsed=now-started
            stamp=node.get_clock().now().to_msg()
            if now-last_imu>=.005:
                m=Imu();m.header.stamp=stamp;m.linear_acceleration.y=-9.805;imu.publish(m);last_imu=now
            if now-last_frame>=.05:
                for pub,array,encoding in [(left,blank if 6<=elapsed<9 else texture,'mono8'),(depth,measured,'16UC1')]:
                    m=Image();m.header.stamp=stamp;m.height=480;m.width=640;m.encoding=encoding
                    m.step=640*array.dtype.itemsize;m.data=array.tobytes();pub.publish(m)
                last_frame=now
            rclpy.spin_once(node,timeout_sec=0)
            time.sleep(.001)
        before=[v for t,v in paths if t-started<6 and v>0]
        after=[v for t,v in paths if t-started>11 and v>0]
        assert before, 'No initial trajectory'
        assert any(v for _,v in events), 'No reset event on tracking loss'
        assert any(v==0 for _,v in paths), 'No empty trajectory on reset'
        assert after, 'Trajectory did not resume after texture returned'
        print(f'PASS: initial path, {len(events)} reset events, cleared path, resumed path; frames={len(paths)}')
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close()
node.destroy_node()
if rclpy.ok():rclpy.shutdown()
