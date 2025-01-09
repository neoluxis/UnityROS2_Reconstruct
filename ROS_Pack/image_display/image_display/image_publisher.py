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


def cv2ros(Img):
    SImg = cv.resize(Img, (640, 480))
    rimg = cv.imencode(".png", SImg)[1].tobytes()
    return rimg


class ImagePublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Image Publisher: %s" % name)
        self.publisher = self.create_publisher(CompressedImage, "image", 10)
        self.time = 1
        self.timer = self.create_timer(self.time, self.timer_callback)
        self.image = cv.imread(
            "/home/neolux/workspace/Unity/ROS_Pack/fxxk_calendar.png"
        )

    def timer_callback(self):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = cv2ros(self.image)
        self.publisher.publish(msg)
        self.get_logger().info("Publishing image")


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy
    node = ImagePublisher("pub1")  # 新建一个节点
    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # 关闭rclpy
