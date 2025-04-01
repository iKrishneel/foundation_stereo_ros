#!/usr/bin/env python

from typing import List
import importlib
import os.path as osp
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from functools import cached_property

from igniter.main import get_full_config
from igniter.builder import build_engine

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError


def depth_xyzmap(depth: np.ndarray, intrinsic: np.ndarray, uvs: np.ndarray = None, zmin: float = 0.1) -> np.ndarray:
    invalid_mask = depth < zmin
    h, w = depth.shape[:2]
    if uvs is None:
        vs, us = np.meshgrid(np.arange(0, h), np.arange(0, w), sparse=False, indexing='ij')
        vs, us = [z.reshape(-1) for z in [vs, us]]
    else:
        uvs = uvs.round().astype(int)
        us = uvs[:, 0]
        vs = uvs[:, 1]

    zs = depth[vs, us]
    xs = (us - intrinsic[0, 2]) * zs / intrinsic[0, 0]
    ys = (vs - intrinsic[1, 2]) * zs / intrinsic[1, 1]
    pts = np.stack((xs.reshape(-1), ys.reshape(-1), zs.reshape(-1)), 1)
    xyz_map = np.zeros((h, w, 3), dtype=np.float32)
    xyz_map[vs, us] = pts
    if invalid_mask.any():
        xyz_map[invalid_mask] = 0
    return xyz_map


class FoundationStereo(Node):
    def __init__(self, name: str = 'foundation_stereo'):
        super().__init__(name)

        self.logger = self.get_logger()
        self.bridge = CvBridge()
        
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.1)
        self.declare_parameter('config', '')
        self.declare_parameter('baseline', 0.1)
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('pub_pcd', False)

        config = self.get_parameter('config').get_parameter_value().string_value
        slop = self.get_parameter('slop').get_parameter_value().double_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.baseline = self.get_parameter('baseline').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        pub_pcd = self.get_parameter('pub_pcd').get_parameter_value().bool_value

        assert osp.isfile(config), f'{config} not found!'
        assert self.baseline >  0.0
        assert self.scale <= 1.0

        config = get_full_config(config)
        if config.driver is not None:
            importlib.import_module(config.driver) 
        self.engine = build_engine(config)

        self.pub_depth = self.create_publisher(Image, 'depth', queue_size)
        if pub_pcd:
            self.pub_pcd = self.create_publisher(PointCloud2, 'points', queue_size)

        sub_left = Subscriber(self, Image, 'left')
        sub_right = Subscriber(self, Image, 'right')
        sub_info = Subscriber(self, CameraInfo, 'info')

        ts = ApproximateTimeSynchronizer([sub_left, sub_right, sub_info], queue_size=queue_size, slop=slop)
        ts.registerCallback(self.callback)

        self.logger.info(f'Node {name} is ready...')

    def callback(self, img_left: Image, img_right: Image, info: CameraInfo = None) -> None:
        header = img_left.header
        img_left, img_right = [self.imgmsg_to_cv2(msg) for msg in [img_left, img_right]]

        self.logger.warn(f'>>> {img_left.shape}')
        
        if self.scale < 1:
            img_left, img_right = [cv.resize(img, fx=self.scale, fy=self.scale, dsize=None) for img in [img_left, img_right]]

        import time; st = time.time()
        disparity = self.engine(img_left, img_right)
        self.logger.info(f'Time {time.time() - st}')

        # intrinsic = np.array([[1952.5, 0, 1104], [0, 1952.5, 621], [0, 0, 1]])
        intrinsic = np.array(info.k).reshape(-1, 3)
        intrinsic[:2] *= self.scale
        im_depth = intrinsic[0, 0] * self.baseline / disparity

        im_depth = im_depth.cpu().numpy() if not isinstance(im_depth, np.ndarray) else im_depth

        if hasattr(self, 'pub_pcd') and self.pub_pcd.get_subscription_count() > 0:
            points = self.to_pcd(im_depth, intrinsic)            

            rgb_values = img_left.reshape(-1, 3)
            rgb_values = (rgb_values[:, 0] << 16) | (rgb_values[:, 1] << 8) | rgb_values[:, 2]

            pcd = create_cloud(header, self.fields, zip(points[:, 0],points[:, 1], points[:, 2], rgb_values))
            pcd.header.frame_id = 'map'
            self.pub_pcd.publish(pcd)

        im_depth = self.bridge.cv2_to_imgmsg(im_depth)
        im_depth.header = header
        self.pub_depth.publish(im_depth)

    def to_pcd(self, depth: np.ndarray, intrinsic: np.ndarray) -> np.ndarray:
         height, width = depth.shape
         u, v = np.meshgrid(np.arange(width), np.arange(height))
         u, v, z = [m.flatten() for m in [u, v, depth]]

         valid = z > 0
         u = u[valid]
         v = v[valid]
         z = z[valid]

         x = (u - intrinsic[0, 2]) * z / intrinsic[0, 0]
         y = (v - intrinsic[1, 2]) * z / intrinsic[1, 1]
         return np.vstack((x, y, z)).T
    
    def imgmsg_to_cv2(self, msg: Image, encoding: str = 'passthrough') -> np.ndarray:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            return image
        except CvBridgeError as e:
            self.logger.error(f'{e}')

    @cached_property
    def fields(self) -> List[PointField]:
        return  [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]


def main(args = None):
    rclpy.init(args=args)
    fs = FoundationStereo()

    rclpy.spin(fs)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
