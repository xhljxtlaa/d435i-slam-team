#!/usr/bin/env python3
"""Fixed metric color scale for display only; never feeds colorized data to VINS."""
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def colorize_depth(z, near, far):
    valid = np.isfinite(z) & (z > 0)
    level = np.zeros(z.shape, np.uint8)
    level[valid] = np.round(255*(1-np.clip((z[valid]-near)/(far-near),0,1))).astype(np.uint8)
    color = cv2.applyColorMap(level, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    color[valid & (z > far)] = (90,90,90)
    color[valid & (z < near)] = (255,255,255)
    return color


class DepthPreview(Node):
    def __init__(self):
        super().__init__('depth_preview')
        self.near = float(self.declare_parameter('near',.18).value)
        self.far = float(self.declare_parameter('far',1.).value)
        self.scale = float(self.declare_parameter('depth_scale',.001).value)
        if not 0 < self.near < self.far:
            raise ValueError('Expected 0 < near < far')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub=self.create_publisher(Image,'/camera/depth_preview',QoSProfile(depth=1))
        self.create_subscription(Image,'/camera/camera/depth/image_rect_raw',self.receive,qos)
        self.last=0.

    def receive(self,m):
        if time.monotonic()-self.last < 1/15:
            return
        self.last=time.monotonic()
        if m.encoding in ('16UC1','mono16'):
            dtype=np.dtype('>u2' if m.is_bigendian else '<u2')
            z=np.frombuffer(m.data,dtype).reshape(m.height,m.step//2)[:,:m.width].astype(np.float32)*self.scale
        elif m.encoding=='32FC1':
            dtype=np.dtype('>f4' if m.is_bigendian else '<f4')
            z=np.frombuffer(m.data,dtype).reshape(m.height,m.step//4)[:,:m.width]
        else:
            return
        color=colorize_depth(z,self.near,self.far)
        legend=np.zeros((62,m.width,3),np.uint8)
        cv2.putText(legend,f'Depth {self.near:.2f}-{self.far:.2f} m | black: invalid | white: too near | gray: far',
            (6,17),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1)
        gradient=np.linspace(self.near,self.far,m.width-20,dtype=np.float32)[None,:]
        legend[24:35,10:-10]=np.repeat(colorize_depth(gradient,self.near,self.far),11,axis=0)
        for value in np.linspace(self.near,self.far,6):
            x=int(10+(value-self.near)/(self.far-self.near)*(m.width-20))
            cv2.putText(legend,f'{value:.2f}',(min(x,m.width-45),53),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
        view=np.vstack([legend,color])
        msg=Image();msg.header=m.header;msg.height=view.shape[0];msg.width=view.shape[1]
        msg.encoding='bgr8';msg.step=msg.width*3;msg.data=view.tobytes()
        self.pub.publish(msg)


if __name__=='__main__':
    rclpy.init();node=DepthPreview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
