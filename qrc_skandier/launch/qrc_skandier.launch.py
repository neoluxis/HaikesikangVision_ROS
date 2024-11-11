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


def generate_launch_description():
    cap_qrc_dev_arg = DeclareLaunchArgument(
        "cap_qrc", default_value="/dev/video0", description="qrcode camera device"
    )

    # 二维码识别的 usb camera
    qrc_usb_cam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("qrc_hobot_usb_cam"),
                "launch/hobot_usb_cam.launch.py",
            )
        ),
        launch_arguments={
            "usb_image_width": "640",
            "usb_image_height": "400",
            "usb_video_device": LaunchConfiguration("cap_qrc"),
        }.items(),
    )

    return LaunchDescription(
        [
            cap_qrc_dev_arg,
            qrc_usb_cam_node,
        ]
    )
