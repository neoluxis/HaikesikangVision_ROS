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
        "cap_qrc", default_value="/dev/video2", description="qrcode camera device"
    )

    # TODO: 不能同时开两个？ : 4 usb port are in the same bus. and the usb camera speed is too high
    # 二维码识别的 usb camera
    # qrc_usb_cam_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("qrc_hobot_usb_cam"),
    #             "launch/hobot_usb_cam.launch.py",
    #         )
    #     ),
    #     launch_arguments={
    #         "usb_image_width": "640",
    #         "usb_image_height": "400",
    #         "usb_framerate": "240",
    #         "usb_pixel_format": "mjpeg2rgb",
    #         "usb_video_device": LaunchConfiguration("cap_qrc"),
    #     }.items(),
    # )

    qrc_usb_cam_node = Node(
        package="qrc_skandier",
        executable="qrc_cam",
        name="qrc_cam",
        output="screen",
    )

    qrc_scanner_hobot_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_codec"),
                "launch/hobot_codec_encode.launch.py",
            )
        ),
        launch_arguments={
            "codec_in_mode": "ros",
            "codec_in_format": "rgb8",
            "codec_out_mode": "ros",
            "codec_sub_topic": "/qrc_image",
            "codec_pub_topic": "/qrc_image_mjpeg",
        }.items(),
    )

    qrc_scanner_node = Node(
        package="qrc_skandier",
        executable="qrc_scanner",
        name="qrc_scanner",
        output="screen",
    )

    qrc_killer_node = Node(
        package="qrc_skandier",
        executable="qrc_cam_killer",
        name="qrc_cam_killer",
        output="screen",
    )

    return LaunchDescription(
        [
            cap_qrc_dev_arg,
            qrc_usb_cam_node,
            qrc_scanner_node,
            # qrc_scanner_hobot_codec_node,
            qrc_killer_node,
        ]
    )
