# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_prefix('hobot_usb_cam'),
        "lib/hobot_usb_cam/config/usb_camera_calibration.yaml")
    print("config_file_path is ", config_file_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_camera_calibration_file_path',
            default_value=TextSubstitution(text=str(config_file_path)),
            description='camera calibration file path'),
        DeclareLaunchArgument(
            'usb_frame_id',
            default_value='default_usb_cam',
            description='image message frame_id'),
        DeclareLaunchArgument(
            'usb_framerate',
            default_value='240',
            description='framerate'),
        DeclareLaunchArgument(
            'usb_image_height',
            default_value='400',
            description='image height'),
        DeclareLaunchArgument(
            'usb_image_width',
            default_value='640',
            description='image width'),
        DeclareLaunchArgument(
            'usb_io_method',
            default_value='mmap',
            description='io_method, mmap/read/userptr'),
        DeclareLaunchArgument(
            'usb_pixel_format',
            default_value='mjpeg',
            description='pixel format, mjpeg'),
        DeclareLaunchArgument(
            'usb_video_device',
            default_value='/dev/video0',
            description='usb camera device'),
        DeclareLaunchArgument(
            'usb_zero_copy',
            default_value='False',
            description='use zero copy or not'),
        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        Node(
            package='qrc_hobot_usb_cam',
            executable='qrc_hobot_usb_cam',
            name='qrc_hobot_usb_cam',
            parameters=[
                {"camera_calibration_file_path": LaunchConfiguration(
                    'usb_camera_calibration_file_path')},
                {"frame_id": LaunchConfiguration('usb_frame_id')},
                {"framerate": LaunchConfiguration('usb_framerate')},
                {"image_height": LaunchConfiguration('usb_image_height')},
                {"image_width": LaunchConfiguration('usb_image_width')},
                {"io_method": LaunchConfiguration('usb_io_method')},
                {"pixel_format": LaunchConfiguration('usb_pixel_format')},
                {"video_device": LaunchConfiguration('usb_video_device')},
                {"zero_copy": LaunchConfiguration('usb_zero_copy')}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
