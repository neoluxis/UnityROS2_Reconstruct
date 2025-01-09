#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QTimer
import cv2 as cv
import numpy as np

class ImageWidget(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)

    def set_image(self, image):
        q_image = self.convert_cv_qt(image)
        pixmap = QPixmap.fromImage(q_image)
        self.setPixmap(pixmap)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv.cvtColor(cv_img, cv.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.width(), self.height(), Qt.KeepAspectRatio)
        return p

def ros2cv(RImg: CompressedImage):
    if RImg is None:
        return None
    np_arr = np.frombuffer(RImg.data, np.uint8)
    cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
    return cv_image


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera_image',
            self.image_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg):
        try:
            cv_image = ros2cv(msg)
            height, width = cv_image.shape[:2]
            half_width = width // 2
            self.left_image = cv_image[:, :half_width]
            self.right_image = cv_image[:, half_width:]
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert image: {e}')

    def timer_callback(self):
        if self.left_image is not None and self.right_image is not None:
            self.left_widget.set_image(self.left_image)
            self.right_widget.set_image(self.right_image)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS2 Image Viewer')
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.left_widget = ImageWidget(self)
        self.right_widget = ImageWidget(self)
        self.layout.addWidget(self.left_widget)
        self.layout.addWidget(self.right_widget)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    image_subscriber = ImageSubscriber()
    main_window = MainWindow()
    image_subscriber.left_widget = main_window.left_widget
    image_subscriber.right_widget = main_window.right_widget
    main_window.show()
    try:
        rclpy.spin(image_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.try_shutdown()
        app.quit()


if __name__ == "__main__":
    main()
