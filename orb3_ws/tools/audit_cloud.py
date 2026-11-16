#!/usr/bin/env python3
"""Validate real cloud messages and invert the pose to check raw-depth geometry."""
import argparse
from collections import OrderedDict
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from capture_config import rotation


def stamp(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--settings',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--seconds',type=float,default=12.)
    args=p.parse_args()
    fs=cv2.FileStorage(args.settings,cv2.FILE_STORAGE_READ)
    K=np.array([[fs.getNode('Camera1.fx').real(),0,fs.getNode('Camera1.cx').real()],
                [0,fs.getNode('Camera1.fy').real(),fs.getNode('Camera1.cy').real()],[0,0,1.]])
    D=np.array([fs.getNode('Camera1.'+k).real() for k in ['k1','k2','p1','p2','k3']])
    factor=fs.getNode('RGBD.DepthMapFactor').real();fs.release()
    rclpy.init();node=rclpy.create_node('orb3_cloud_audit')
    poses,depths,pending=OrderedDict(),OrderedDict(),OrderedDict()
    counts={k:[] for k in ['surface','sparse','current_depth']}
    frames=set();z_errors=[];pixel_errors=[];paired=set();failures=[]
    def bounded(d,key,value,limit):
        d[key]=value
        while len(d)>limit:d.popitem(last=False)
    def pose(m):bounded(poses,stamp(m),m.pose,150)
    def depth(m):
        dtype='<u2' if m.encoding=='16UC1' else '<f4'
        a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,
                     strides=(m.step,np.dtype(dtype).itemsize)).copy()
        bounded(depths,stamp(m),a,60)
    def cloud(m,key):
        frames.add(m.header.frame_id)
        if m.point_step!=16 or len(m.data)!=m.width*m.height*16:
            failures.append('Unexpected PointCloud2 layout');return
        a=np.frombuffer(m.data,dtype='<f4').reshape(-1,4)
        if not np.isfinite(a).all():failures.append('Non-finite cloud values')
        if len(a) and (a[:,3].min()<0 or a[:,3].max()>255):failures.append('Invalid intensity range')
        if len(a)>250000:failures.append('Map capacity exceeded')
        counts[key].append(len(a))
        if key=='current_depth':bounded(pending,stamp(m),a.copy(),5)
    qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE)
    map_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[node.create_subscription(PoseStamped,'/orb3/pose',pose,qos),
                   node.create_subscription(Image,'/camera/camera/depth/image_rect_raw',depth,qos)]
    for key,topic in [('surface','map_cloud'),('sparse','map_points'),('current_depth','depth_points')]:
        subscriptions.append(node.create_subscription(PointCloud2,'/orb3/'+topic,
            lambda m,k=key:cloud(m,k),map_qos))
    deadline=time.monotonic()+args.seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        for t,points in list(pending.items()):
            if t in paired or t not in poses or t not in depths or len(points)==0:continue
            sample=points[:200,:3].astype(float);pose_msg=poses[t]
            R=rotation(pose_msg.orientation)
            translation=np.array([pose_msg.position.x,pose_msg.position.y,pose_msg.position.z])
            camera=(sample-translation)@R
            pixels=cv2.projectPoints(camera,np.zeros(3),np.zeros(3),K,D)[0][:,0,:]
            rounded=np.rint(pixels).astype(int)
            h,w=depths[t].shape
            if np.any(rounded[:,0]<0) or np.any(rounded[:,0]>=w) or np.any(rounded[:,1]<0) or np.any(rounded[:,1]>=h):
                failures.append('Depth points project outside image');continue
            measured=depths[t][rounded[:,1],rounded[:,0]]/factor
            z_errors.append(float(np.max(np.abs(camera[:,2]-measured))))
            pixel_errors.append(float(np.max(np.abs(pixels-rounded))))
            paired.add(t)
    result={'cloud_message_counts':{k:len(v) for k,v in counts.items()},
            'cloud_point_counts':{k:{'min':min(v),'max':max(v)} if v else None for k,v in counts.items()},
            'frames':sorted(frames),'paired_pose_depth_cloud_frames':len(paired),
            'max_backprojected_depth_error_metres':max(z_errors) if z_errors else None,
            'max_reprojected_pixel_rounding_error':max(pixel_errors) if pixel_errors else None,
            'failures':failures}
    result['passed']=(all(v and max(v)>0 for v in counts.values()) and frames=={'orb3_map'}
                      and len(paired)>=2 and max(z_errors)<1e-4 and max(pixel_errors)<.02 and not failures)
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
