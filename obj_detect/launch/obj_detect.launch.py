from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

import subprocess
import re
import cv2


def get_max_fps(device):
    """使用 v4l2-ctl 查询摄像头支持的最大帧率"""
    try:
        # 调用 v4l2-ctl 列出支持的格式和帧率
        result = subprocess.run(
            ["v4l2-ctl", "--list-formats-ext", "-d", f"/dev/{device}"],
            capture_output=True,
            text=True,
            check=True,
        )
        output = result.stdout

        # 使用正则表达式匹配最大帧率
        fps_matches = re.findall(r"Interval: .*\((\d+\.\d+) fps\)", output)
        if not fps_matches:
            print(f"未能找到 {device} 的帧率信息")
            return None

        # 获取最高帧率
        max_fps = max(float(fps) for fps in fps_matches)
        return max_fps

    except subprocess.CalledProcessError:
        print(f"无法访问设备 {device}")
        return None


def find_camera(dev_nodes=["video0", "video2"]):
    """
    根据摄像头的最大帧率，返回两个摄像头设备节点
    返回顺序：帧率高的在前，作为二维码摄像头（因为他是黑白的）
    """
    fps1 = get_max_fps(dev_nodes[0])
    fps2 = get_max_fps(dev_nodes[1])
    if fps1 > fps2:
        return dev_nodes[0], dev_nodes[1]
    else:
        return dev_nodes[1], dev_nodes[0]


def generate_launch_description():
    return LaunchDescription(
        [
            #           # 串口发送
            Node(
                package="obj_detect",
                executable="obj_serial",
                name="obj_serial",
            ),
            # 算法
            # Node(
            #     package="dnn_node_example",
            #     executable="example",
            #     parameters=[
            #         {"config_file": '/root/dev_ws/gz/dnn/gz1017.json'},
            #         {"dump_render_img": LaunchConfiguration(
            #             'dnn_example_dump_render_img')},
            #         {"feed_type": 1},
            #         {"is_shared_mem_sub": 1},
            #         {"msg_pub_topic_name": LaunchConfiguration(
            #             "dnn_example_msg_pub_topic_name")}
            #     ],
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("dnn_node_example"),
                        "launch/dnn_node_example.launch.py",
                    )
                ),
                launch_arguments={
                    # 'websocket_image_topic': '/image',
                    # 'websocket_image_type': 'mjpeg',
                    # 'websocket_smart_topic': LaunchConfiguration("dnn_example_msg_pub_topic_name")
                    "dnn_example_config_file": "/root/dev_ws/neo_qrcode/dnn/task_obj.json",
                }.items(),
            ),
            # web展示pkg
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("websocket"),
                        "launch/websocket.launch.py",
                    )
                ),
                launch_arguments={
                    "websocket_image_topic": "/image",
                    "websocket_image_type": "mjpeg",
                    "websocket_smart_topic": LaunchConfiguration(
                        "dnn_example_msg_pub_topic_name"
                    ),
                }.items(),
            ),
        ]
    )
