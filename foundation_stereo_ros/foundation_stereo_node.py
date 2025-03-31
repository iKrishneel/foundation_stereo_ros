#!/usr/bin/env python

import importlib
import os.path as osp
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node

from igniter.main import get_full_config
from igniter.builder import build_engine

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
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
        self.declare_parameter('config', '') # , description='Configuration file')        
        self.declare_parameter('baseline', 0.1) # , description='Distance between the stereo camera')
        self.declare_parameter('scale', 1.0) # , description='Image downsize')
        self.declare_parameter('pub_pcd', False) #, description='Publish pointcloud')

        config = self.get_parameter('config').get_parameter_value().string_value
        slop = self.get_parameter('slop').get_parameter_value().double_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.baseline = self.get_parameter('baseline').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        pub_pcd = self.get_parameter('pub_pcd').get_parameter_value().bool_value

        assert osp.isfile(config), f'{config} not found!'
        assert self.baseline >  0.0
        assert self.scale <= 1.0

        self.pub_depth = self.create_publisher(Image, 'depth', queue_size)
        if pub_pcd:
            self.pub_pcd = self.create_publisher(PointCloud2, 'points', queue_size)

        sub_left = Subscriber(self, Image, 'left')
        sub_right = Subscriber(self, Image, 'right')
        # sub_info = Subscriber(self, CameraInfo, 'info')

        ts = ApproximateTimeSynchronizer([sub_left, sub_right, ], queue_size=queue_size, slop=slop)
        ts.registerCallback(self.callback)

        config = get_full_config(config)
        if config.driver is not None:
            importlib.import_module(config.driver) 
        self.engine = build_engine(config)

        self.logger.info(f'Node {name} is ready...')

    def callback(self, img_left: Image, img_right: Image, info: CameraInfo = None) -> None:
        header = img_left.header
        img_left, img_right = [self.imgmsg_to_cv2(msg) for msg in [img_left, img_right]]

        if self.scale < 1:
            img_left, img_right = [cv.resize(img, fx=self.scale, fy=self.scale, dsize=None) for img in [img_left, img_right]]

        disparity = self.engine(img_left, img_right)

        im_depth = (disparity.float()).cpu().numpy()
        im_depth = (((im_depth - im_depth.min()) / (im_depth.max() - im_depth.min()) ) * 255).astype(np.uint8)
        # intrinsic = np.array(info.k).reshape(-1, 3)
        # intrinsic[:2] *= self.scale
        # im_depth = intrinsic[0, 0] * self.baseline / disparity

        if not hasattr(self, 'pub_pcd'):
            pass

        im_depth = self.bridge.cv2_to_imgmsg(im_depth)
        im_depth.header = header
        self.pub_depth.publish(im_depth)

    def imgmsg_to_cv2(self, msg: Image, encoding: str = 'passthrough') -> np.ndarray:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            return image
        except CvBridgeError as e:
            self.logger.error(f'{e}')


def main(args = None):
    rclpy.init(args=args)
    fs = FoundationStereo()

    rclpy.spin(fs)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
