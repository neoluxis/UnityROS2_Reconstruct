import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from camera_msgs.msg import BinocularImage

import cv2 as cv
import numpy as np
import os
import time


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


class ImageSplit(Node):
    def __init__(self, node_name):
        super().__init__(f"image_split_{node_name}")
        self.image_topic = "camera_image"
        
        self.image_sub = self.create_subscription(
            CompressedImage, self.image_topic, self.image_callback, 10
        )

        self.image_pub = self.create_publisher(BinocularImage, "binocular_image", 10)

        self.get_logger().info(f"Node {node_name} has been started")
        self.get_logger().info(f"Subscribed to {self.image_topic}")
        self.get_logger().info("Publishing to binocular_image")

    def image_callback(self, msg):
        self.get_logger().info("Receiving image")
        cv_image = ros2cv(msg)
        # cv.imshow("Image", cv_image)
        # cv.waitKey(1)

        left_image = cv_image[:, : cv_image.shape[1] // 2]
        right_image = cv_image[:, cv_image.shape[1] // 2 :]

        msg = BinocularImage()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "binocular_image"
        msg.leftocular = CompressedImage()
        msg.leftocular.data = cv2ros(left_image)
        msg.leftocular.format = "png"
        msg.rightocular = CompressedImage()
        msg.rightocular.data = cv2ros(right_image)
        msg.rightocular.format = "png"

        self.image_pub.publish(msg)
        self.get_logger().info("Publishing image")
        cv.imshow("Left Image", left_image)
        cv.imshow("Right Image", right_image)
        if cv.waitKey(1) == ord("s"):
            cv.imwrite(f"left.png", left_image)
            cv.imwrite(f"right.png", right_image)


def main():
    rclpy.init()
    node = ImageSplit("1")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
