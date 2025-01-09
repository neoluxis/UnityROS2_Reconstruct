#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
import cv2 as cv

import numpy as np


def ros2cv(RImg: CompressedImage):
    if RImg is None:
        return None
    np_arr = np.frombuffer(RImg.data, np.uint8)
    cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
    return cv_image


class ImageDisplay(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Image Display: %s" % name)
        self.subscription = self.create_subscription(
            CompressedImage, "camera_image", self.image_callback, 10
        )

    def image_callback(self, msg):
        self.get_logger().info("Receiving image")
        self.get_logger().info("Image length: %d" % len(msg.data))
        cv_image = ros2cv(msg)
        cv.imshow("Image", cv_image)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy
    node = ImageDisplay("disp1")  # 新建一个节点
    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # 关闭rclpy
