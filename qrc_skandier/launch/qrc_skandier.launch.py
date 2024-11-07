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
    return LaunchDescription(
        [
#           # 串口发送
            Node(
                package="gz",
                # namespace="color",
                executable="gz_serial",
                name="gz_serial",
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
                        get_package_share_directory('dnn_node_example'),
                        'launch/dnn_node_example.launch.py')),
                launch_arguments={
                    # 'websocket_image_topic': '/image',
                    # 'websocket_image_type': 'mjpeg',
                    # 'websocket_smart_topic': LaunchConfiguration("dnn_example_msg_pub_topic_name")
                    'dnn_example_config_file': '/root/dev_ws/gz/dnn/gz_1017.json',
                }.items()
            ),

            # web展示pkg
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('websocket'),
                        'launch/websocket.launch.py')),
                launch_arguments={
                    'websocket_image_topic': '/image',
                    'websocket_image_type': 'mjpeg',
                    'websocket_smart_topic': LaunchConfiguration("dnn_example_msg_pub_topic_name")
                }.items()
            )
        ]
    )
