#!/usr/bin/env python3
"""Fit stereo calibration, validate on held-out poses, then optionally apply."""
import argparse
import json
import re
import shutil
import time
from pathlib import Path

import cv2
import numpy as np


def solve(data):
    cv2.setNumThreads(2)
    left = data['left'].astype(np.float32)
    right = data['right'].astype(np.float32)
    obj = data['object_points'].astype(np.float32)
    size = tuple(int(v) for v in data['image_size'])
    if len(left) < 25:
        raise ValueError(f'Need at least 25 diverse pairs, got {len(left)}')
    indices = np.random.default_rng(42).permutation(len(left))
    test, train = indices[:max(5,len(left)//5)], indices[max(5,len(left)//5):]
    K0 = np.array([[386.118,0,322.364],[0,386.118,235.613],[0,0,1.]])
    criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,100,1e-8)
    def fit(ids):
        objects = [obj for _ in ids]
        cameras=[]
        for observations in [left,right]:
            rms,K,D,_,_=cv2.calibrateCamera(objects,[observations[i] for i in ids],
                size,K0.copy(),np.zeros(5),
                flags=cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_FIX_K3,criteria=criteria)
            cameras.append((rms,K,D))
        rms,K1,D1,K2,D2,R,T,_,_=cv2.stereoCalibrate(objects,
            [left[i] for i in ids],[right[i] for i in ids],
            cameras[0][1],cameras[0][2],cameras[1][1],cameras[1][2],size,
            flags=cv2.CALIB_FIX_INTRINSIC,criteria=criteria)
        return cameras,R,T,rms
    cameras,R,T,train_rms=fit(train)
    holdout=[]
    for i in test:
        _,rv,tv=cv2.solvePnP(obj,left[i],cameras[0][1],cameras[0][2])
        Rb=cv2.Rodrigues(rv)[0]
        rvr=cv2.Rodrigues(R@Rb)[0]
        tvr=R@tv+T
        predicted=cv2.projectPoints(obj,rvr,tvr,cameras[1][1],cameras[1][2])[0]
        holdout.append(float(np.sqrt(np.mean(np.sum((predicted-right[i])**2,axis=2)))))
    holdout_rms=float(np.sqrt(np.mean(np.square(holdout))))
    cameras,R,T,rms=fit(range(len(left)))
    centers=np.array([c[:,0,:].mean(axis=0)/np.array(size) for c in left])
    coverage=np.ptp(centers,axis=0)
    normals=[]
    areas=[]
    for c in left:
        _,rv,_=cv2.solvePnP(obj,c,cameras[0][1],cameras[0][2])
        normals.append(cv2.Rodrigues(rv)[0][:,2])
        areas.append(cv2.contourArea(cv2.convexHull(c)))
    normals_span=np.ptp(normals,axis=0)
    scale_ratio=float(np.sqrt(max(areas)/min(areas)))
    baseline=float(np.linalg.norm(T))
    errors=[]
    if max(x[0] for x in cameras)>.7:errors.append('Camera reprojection RMS exceeds 0.7 px')
    if rms>.8:errors.append('Stereo reprojection RMS exceeds 0.8 px')
    if holdout_rms>1.2:errors.append('Held-out stereo projection RMS exceeds 1.2 px')
    if abs(baseline/.0499301-1)>.05:errors.append('Baseline differs from SDK by over 5%; check square size and target flatness')
    if any(coverage<.20):errors.append('Board center coverage is too narrow; include all image quadrants')
    if any(normals_span[:2]<.35):errors.append('Insufficient horizontal/vertical tilt diversity')
    if scale_ratio<1.35:errors.append('Insufficient distance/scale diversity')
    for _,K,D in cameras:
        if not np.isfinite(K).all() or not np.isfinite(D).all() or not (250<K[0,0]<600 and 250<K[1,1]<600):
            errors.append('Implausible intrinsics')
    report=dict(samples=len(left),square_mm=float(data['square_mm']),image_size=size,
        camera_rms_px=[float(x[0]) for x in cameras],stereo_rms_px=float(rms),
        heldout_stereo_rms_px=holdout_rms,heldout_per_view_px=holdout,
        baseline_m=baseline,center_coverage=coverage.tolist(),
        board_normal_span=normals_span.tolist(),scale_ratio=scale_ratio,
        K=[x[1].tolist() for x in cameras],D=[x[2].flatten().tolist() for x in cameras],
        R_cam1_from_cam0=R.tolist(),T_cam1_from_cam0=T.flatten().tolist(),
        accepted=not errors,rejection_reasons=errors,
        scope='Stereo intrinsics and stereo extrinsics only; IMU calibration not estimated')
    return report


def write_candidates(report,folder,base):
    size=report['image_size']
    for idx,name in enumerate(['left','right']):
        K=np.array(report['K'][idx]);D=report['D'][idx]
        text=f'%YAML:1.0\n---\nmodel_type: PINHOLE\ncamera_name: {name}\nimage_width: {size[0]}\nimage_height: {size[1]}\ndistortion_parameters:\n'
        text+=''.join(f'   {key}: {value:.15g}\n' for key,value in zip(['k1','k2','p1','p2'],D))
        text+='projection_parameters:\n'+''.join(f'   {key}: {value:.15g}\n' for key,value in zip(['fx','fy','cx','cy'],[K[0,0],K[1,1],K[0,2],K[1,2]]))
        (folder/f'{name}.yaml').write_text(text)
    fs=cv2.FileStorage(str(base),cv2.FILE_STORAGE_READ)
    body_left=fs.getNode('body_T_cam0').mat();fs.release()
    stereo=np.eye(4);stereo[:3,:3]=report['R_cam1_from_cam0'];stereo[:3,3]=report['T_cam1_from_cam0']
    body_right=body_left@np.linalg.inv(stereo)
    text=base.read_text()
    block='body_T_cam1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: d\n   data: ['+', '.join(f'{v:.15g}' for v in body_right.flatten())+']\n'
    text=re.sub(r'body_T_cam1: !!opencv-matrix.*?(?=\n\n)',lambda m:block,text,flags=re.S)
    (folder/'stereo_imu.yaml').write_text(text)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('session',type=Path)
    p.add_argument('--apply',action='store_true')
    args=p.parse_args()
    data=np.load(args.session/'observations.npz')
    report=solve(data)
    print(json.dumps(report,indent=2))
    (args.session/'stereo_calibration_report.json').write_text(json.dumps(report,indent=2))
    if not report['accepted']:
        raise SystemExit('Quality checks failed; active VINS configuration unchanged')
    candidates=args.session/'validated_config';candidates.mkdir(exist_ok=True)
    active=Path(__file__).resolve().parents[1] / 'config/d435i'
    write_candidates(report,candidates,active/'stereo_imu.yaml')
    if args.apply:
        backup=active/('backup_'+time.strftime('%Y%m%d_%H%M%S'));backup.mkdir()
        for name in ['left.yaml','right.yaml','stereo_imu.yaml']:
            shutil.copy2(active/name,backup/name)
            shutil.copy2(candidates/name,active/name)
        print('Applied validated stereo calibration; restart VINS and clear accumulated map')


if __name__=='__main__':
    main()
