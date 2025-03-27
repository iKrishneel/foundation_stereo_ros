#!/usr/bin/env python

import cv2 as cv
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError


class FoundationStereo(Node):
    def __init__(self, name: str = 'foundation_stereo'):
        super().__init__(name)

        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.1)

        slop = self.get_parameter('slop').get_parameter_value().double_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

        sub_left = Subscriber(self, Image, 'left')
        sub_right = Subscriber(self, Image, 'right')

        ts = ApproximateTimeSynchronizer([sub_left, sub_right], queue_size=queue_size, slop=slop)
        ts.registerCallback(self.callback)

        self.logger = self.get_logger()
        self.bridge = CvBridge()

    def callback(self, img_left: Image, img_right: Image) -> None:
        try:
            im_left, im_right = [self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') for msg in [img_left, img_right]]
        except CvBridgeError as e:
            self.logger.error(f'{e}')


def main(args=None):
    rclpy.init(args=args)
    fs = FoundationStereo()

    rclpy.spin(fs)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
