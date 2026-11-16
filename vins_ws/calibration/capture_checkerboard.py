#!/usr/bin/env python3
"""Synchronized stereo checkerboard capture with pose diversity and live preview."""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    cv2.setNumThreads(2)
    p = argparse.ArgumentParser()
    p.add_argument('--cols', type=int, default=8)
    p.add_argument('--rows', type=int, default=6)
    p.add_argument('--square-mm', type=float, default=24.4)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=True)
    if (folder/'observations.npz').exists():
        raise RuntimeError('Output already contains observations; use a new directory')
    rclpy.init()
    node = rclpy.create_node('checkerboard_capture')
    buffers = [{}, {}]
    subs = []
    for i in range(2):
        def cb(m, side=i):
            stamp = m.header.stamp.sec*1000000000 + m.header.stamp.nanosec
            im = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width].copy()
            buffers[side][stamp] = im
            while len(buffers[side]) > 20:
                del buffers[side][next(iter(buffers[side]))]
        subs.append(node.create_subscription(Image,
            f'/camera/camera/infra{i+1}/image_rect_raw', cb, qos_profile_sensor_data))
    pattern = (args.cols, args.rows)
    obj = np.zeros((args.cols*args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[:args.cols, :args.rows].T.reshape(-1, 2)*args.square_mm/1000
    left, right, stamps, poses, descriptors = [], [], [], [], []
    last_processed = last_saved = last_status = 0
    state = 'Place entire checkerboard in BOTH views'
    cv2.namedWindow('D435i Stereo Calibration - Q to pause', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('D435i Stereo Calibration - Q to pause', 1280, 580)
    def save():
        np.savez_compressed(folder/'observations.npz', left=np.array(left),
            right=np.array(right), stamps=np.array(stamps), object_points=obj,
            image_size=np.array(size), square_mm=args.square_mm, pattern=pattern)
        info = dict(samples=len(left), pattern=list(pattern), square_mm=args.square_mm,
            image_size=list(size), grid_cells=len(set((d[0],d[1]) for d in descriptors)))
        (folder/'capture_status.json').write_text(json.dumps(info, indent=2))
        print(json.dumps(info), flush=True)
    size = (640,480)
    try:
        while rclpy.ok():
            for _ in range(12):
                rclpy.spin_once(node, timeout_sec=.005)
            common = buffers[0].keys() & buffers[1].keys()
            if not common:
                cv2.waitKey(1)
                continue
            stamp = max(common)
            if stamp <= last_processed:
                continue
            last_processed = stamp
            ims = [b[stamp] for b in buffers]
            size = (ims[0].shape[1], ims[0].shape[0])
            results = [cv2.findChessboardCornersSB(im, pattern,
                cv2.CALIB_CB_NORMALIZE_IMAGE) for im in ims]
            ok = all(result[0] for result in results)
            if ok:
                c0, c1 = [x[1] for x in results]
                # In rectified D435i stereo, corresponding rows should be close.
                if np.median(abs(c0[:,0,1]-c1[:,0,1])) > 3:
                    c1 = c1[::-1].copy()
                aligned = np.median(abs(c0[:,0,1]-c1[:,0,1])) < 3
                K = np.array([[386.118,0,322.364],[0,386.118,235.613],[0,0,1.]])
                solved, rv, tv = cv2.solvePnP(obj, c0, K, np.zeros(5))
                R = cv2.Rodrigues(rv)[0]
                diverse = not poses or all(
                    np.linalg.norm(tv-t) > .04 or
                    np.arccos(np.clip((np.trace(R@r.T)-1)/2,-1,1)) > np.deg2rad(10)
                    for r,t in poses)
                blur = min(cv2.Laplacian(im,cv2.CV_64F).var() for im in ims)
                edge = min(float(np.min(c)) for c in (c0,c1)) > 8
                edge = edge and all(np.max(c[:,0,0])<size[0]-8 and np.max(c[:,0,1])<size[1]-8 for c in (c0,c1))
                if solved and aligned and diverse and edge and blur>25 and time.monotonic()-last_saved>1:
                    left.append(c0);right.append(c1);stamps.append(stamp);poses.append((R,tv))
                    center=c0[:,0,:].mean(axis=0)/np.array(size)
                    descriptors.append((int(center[0]*3),int(center[1]*3)))
                    for side,im in enumerate(ims):
                        cv2.imwrite(str(folder/f'{len(left):03d}_{side}.png'),im)
                    save();last_saved=time.monotonic()
                    state='Accepted! Change position, distance AND tilt'
                elif not aligned:
                    state='Corner ordering/row mismatch - rotate board slightly'
                elif blur<=25:
                    state='Image blurred - hold board/camera still briefly'
                elif not edge:
                    state='Keep ALL corners away from image edges'
                else:
                    state='Need a different pose: move nearer/farther or tilt'
                corners=[c0,c1]
            else:
                state='Board not found in both images - show full board'
                corners=[result[1] if result[0] else None for result in results]
            if time.monotonic()-last_status>5:
                print(f'STATUS samples={len(left)} detected={ok}: {state}',flush=True)
                last_status=time.monotonic()
            previews=[]
            for im,c in zip(ims,corners):
                view=cv2.cvtColor(im,cv2.COLOR_GRAY2BGR)
                if c is not None:cv2.drawChessboardCorners(view,pattern,c,True)
                previews.append(view)
            canvas=np.vstack([np.zeros((85,size[0]*2,3),np.uint8),np.hstack(previews)])
            cv2.putText(canvas,f'{len(left)} accepted stereo pairs | Target: 35-50 diverse poses | Square {args.square_mm} mm',
                (12,26),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,160),2)
            cv2.putText(canvas,state,(12,57),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
            cv2.imshow('D435i Stereo Calibration - Q to pause',canvas)
            if cv2.waitKey(1)&255==ord('q') or len(left)>=50:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if left:save()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
