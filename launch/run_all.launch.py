import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

import subprocess
import re


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
    return sorted(dev_nodes, key=lambda x: get_max_fps(x), reverse=True)


def generate_launch_description():
    cap_qrc_devnode, cap_objdet_devnode = find_camera()
    cap_qrc_devnode = "/dev/" + cap_qrc_devnode
    cap_objdet_devnode = "/dev/" + cap_objdet_devnode

    config_file_launch_arg = DeclareLaunchArgument(
        "dnn_example_config_file",
        default_value=TextSubstitution(text="/root/dev_ws/appli/dnn/task_obj.json"),
    )

    obj_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("obj_detect"),
                "launch/obj_detect.launch.py",
            )
        ),
        launch_arguments={
            "cap_objdet": cap_objdet_devnode,
            "dnn_example_config_file": LaunchConfiguration("dnn_example_config_file"),
        }.items(),
    )

    qrc_skandier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("qrc_skandier"),
                "launch/qrc_skandier.launch.py",
            )
        ),
        launch_arguments={
            "cap_qrc": cap_qrc_devnode,
        }.items(),
    )

    return LaunchDescription(
        [
            config_file_launch_arg,
            obj_detection,
            qrc_skandier,
        ]
    )
