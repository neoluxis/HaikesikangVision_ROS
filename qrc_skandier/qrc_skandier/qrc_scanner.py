import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

import cv2 as cv
import numpy as np
import pyzbar.pyzbar as pyzbar
import time
import datetime


from utils.neo_img_trans import ros2cv


class QrcScanner(Node):
    def __init__(self, name):
        super().__init__(name)

        # self.qrc_img_sub = self.create_subscription(
        #     Image,
        #     "qrc_image",
        #     # "image_mjpeg",
        #     self.scan_code,
        #     10,
        # )
        self.qrc_img_sub = self.create_subscription(
            CompressedImage,
            "qrc_image",
            # "image_mjpeg",
            self.scan_code,
            10,
        )
        self.res_pub = self.create_publisher(String, "qrc_result", 10)
        self.show = True
        self.shutdown_sub = self.create_subscription(
            String,
            "kill_qrc",
            self.shutdown,
            10,
        )

    def scan_code(self, msg):
        t0 = time.time()
        cv_img = ros2cv(msg)
        # cv_img = msg.data
        # cv_img = np.reshape(cv_img, (400, 640, 3))
        # cv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)
        # cv_img = cv.resize(cv_img, (0,0),fx=0.35, fy=0.35)
        # self.get_logger().info(f"{len(msg.data)}")
        # self.get_logger().info(f"{cv_img.shape}")
        # cv_img = msg.data
        if cv_img is None:
            # self.res_pub.publish(String(data="Get None image"))
            self.get_logger().info("Get None image")
            return
        decoded_objects = pyzbar.decode(cv_img)
        if decoded_objects:
            text = decoded_objects[0].data.decode("utf-8")
            self.get_logger().info(f"Detected: {text}")
            self.res_pub.publish(String(data=text))
        self.get_logger().info(f"Scan time: {time.time() - t0:.3f}")
        # cv.imshow("qrc", cv_img)
        # cv.waitKey(1)
        # TODO: Remove this part showing the image instead, using a flask server
        # if self.show:
        #     try:
        #         cv.imshow("qrc", cv_img)
        #         if cv.waitKey(1) == ord("q"):
        #             cv.destroyAllWindows()
        #             self.show = False
        #     except Exception as e:
        #         self.get_logger().info(f"Show image error: {e}")
        #         self.show = False

    def shutdown(self, msg):
        self.get_logger().info(f"Shutdown: {msg.data}")
        self.destroy_node()
        self.get_logger().info(f"Destroyed node {self.get_name()}")
        rclpy.shutdown()


def main():
    rclpy.init()
    qrc_scanner = QrcScanner("qrc_cam")
    rclpy.spin(qrc_scanner)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
