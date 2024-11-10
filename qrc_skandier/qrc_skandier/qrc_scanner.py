import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
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

        self.qrc_img_sub = self.create_subscription(
            Image,
            "qrc_image",
            # "image_mjpeg",
            self.scan_code,
            10,
        )
        self.res_pub = self.create_publisher(String, "qrc_result", 10)
        self.show = True

    def scan_code(self, msg):
        cv_img = ros2cv(msg)
        if cv_img is None:
            self.res_pub.publish(String(data="Get None image"))
            return
        t0 = time.time()
        decoded_objects = pyzbar.decode(cv_img)
        if decoded_objects:
            text = decoded_objects[0].data.decode("utf-8")
            self.get_logger().info(f"Detected: {text}")
            self.res_pub.publish(String(data=text))
        self.get_logger().info(f"Scan time: {time.time() - t0:.3f}")
        # TODO: Remove this part showing the image instead, using a flask server
        if self.show:
            cv.imshow("qrc", cv_img)
            if cv.waitKey(1) == ord("q"):
                cv.destroyAllWindows()
                self.show = False


def main():
    rclpy.init()
    qrc_scanner = QrcScanner("qrc_cam")
    rclpy.spin(qrc_scanner)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
