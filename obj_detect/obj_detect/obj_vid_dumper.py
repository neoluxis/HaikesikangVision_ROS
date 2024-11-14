import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

import subprocess
import os
import time

import cv2 as cv
import numpy as np
import datetime


class ObjVidDumper(Node):
    def __init__(self, name="1"):
        super().__init__(f"obj_vid_dumper_{name}")
        self.get_logger().info(f"Init video dumper, node {name}")

        self.declare_parameter("video_dir", "/root/dev_ws/appli/_tmp_videos/")
        self.declare_parameter("video_fps", 20)
        self.declare_parameter("video_width", 640)
        self.declare_parameter("video_height", 480)
        self.declare_parameter("video_fourcc", "MJPG")
        self.declare_parameter("video_update_time", 10)

        os.makedirs(
            self.get_parameter("video_dir").get_parameter_value().string_value,
            exist_ok=True,
        )
        self.timer_vid_fd_update = self.create_timer(
            self.get_parameter("video_update_time").get_parameter_value().integer_value,
            self.timer_update_fd,
        )

        self.video_fn = None
        self.video_fd = None

        self.image_sub = self.create_subscription(
            CompressedImage, "image", self.image_callback, 10
        )

    def get_fd_fname(self):
        fn = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        fn = os.path.join(
            self.get_parameter("video_dir").get_parameter_value().string_value, fn
        )
        fn = fn + ".avi"
        self.video_fn = fn
        self.get_logger().info(f"Video file updated to: {fn}")
        return fn

    def timer_update_fd(self):
        """
        更新视频文件描述
        """
        if self.video_fd is not None:
            self.video_fd.release()
            self.get_logger().info(f"Video file released: {self.video_fn}")

        self.video_fd = cv.VideoWriter(
            self.get_fd_fname(),
            cv.VideoWriter.fourcc(
                *self.get_parameter("video_fourcc").get_parameter_value().string_value
            ),
            self.get_parameter("video_fps").get_parameter_value().integer_value,
            (
                self.get_parameter("video_width").get_parameter_value().integer_value,
                self.get_parameter("video_height").get_parameter_value().integer_value,
            ),
        )
        self.get_logger().info(f"Video file opened: {self.video_fn}")

    def dump_frame(self, frame, s=[0]):
        """
        保存一帧到视频
        """
        if self.video_fd is None:
            return
        s[0] += 1
        if s[0] % 5 == 0:
            if self.video_fd is not None:
                self.video_fd.write(frame)
                self.get_logger().info(f"Write frame to video: {self.video_fn}")
            else:
                self.get_logger().info(f"Video file not opened: {self.video_fn}")
        else:
            self.get_logger().info(f"Skip frame write to video: {self.video_fn}")
        s[0] = s[0] % 5

    def image_callback(self, msg):
        """
        图像消息回调
        """
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        self.dump_frame(frame)
        


def main(args=None):
    rclpy.init(args=args)
    obj_vid_dumper = ObjVidDumper()
    rclpy.spin(obj_vid_dumper)
    obj_vid_dumper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
