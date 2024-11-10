# Changelog for package hobot_usb_cam

tros_2.3.0 (2024-08-21)
------------------
1. 修复自适应USB设备号失败的问题。

tros_2.2.0 (2024-04-01)
------------------
1. 适配ros2 humble零拷贝。
2. 新增中英双语README。
3. 零拷贝通信使用的qos的Reliability由RMW_QOS_POLICY_RELIABILITY_RELIABLE（rclcpp::QoS()）变更为RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT（rclcpp::SensorDataQoS()）。

tros_2.1.2 (2024-3-4)
------------------
1. 发布图像编码方式`pixel_format`配置项删除`mjpeg-compressed`配置选项，统一使用`mjpeg`配置选项指定发布`jpeg`压缩格式图片。
2. `jpeg`压缩格式图片使用的数据类型由`sensor_msgs::msg::Image`变更为`sensor_msgs::msg::CompressedImage`。
3. 配置文件路径由`/opt/tros/lib`变更为`/opt/tros/${TROS_DISTRO}/lib`。

tros_2.1.1 (2024-1-18)
------------------
1. 增加支持websocket浏览视频的launch。
2. 增加对pixel_format配置与usb camera支持的format校验功能，当没有匹配的format突出node并提示客户更换合适pixel_format;

tros_2.1.0 (2023-12-18)
------------------
1. 重构hobot_usb_cam的这个代码，参考bosch的usb camera的代码，支持mjpeg,rgb,yuv等数据格式的输出。
2. 合并自适应USB设备号失败的问题。增加设备初始化状态判断，解决非USB摄像头打开设备号成功导致的误识别问题。
3. 增加零拷贝的支持。

tros_2.0.1rc1 (2023-07-12)
------------------
1. 修复自适应USB设备号失败的问题。增加设备初始化状态判断，解决非USB摄像头打开设备号成功导致的误识别问题。

tros_2.0.1 (2023-07-05)
------------------
1. 自适应USB设备号。当使用用户指定的设备号打开设备失败时，遍历`/dev/`路径下的`video`设备并尝试打开，直到打开成功或者遍历结束。

tros_2.0.0rc1 (2023-05-23)
------------------
1. 修复usb_camera launch启动失败问题

tros_2.0.0 (2023-05-11)
------------------
1. 更新package.xml，支持应用独立打包
2. 更新应用启动launch脚本

tros_1.1.6a (2023-02-16)
------------------
1. README增加x86平台编译使用说明

tros_1.1.3 (2022-11-16)
------------------
1. 修改readme中参数zero_copy默认值为“True”

tros_1.1.2 (2022-09-28)
------------------
1. 优化hobot_usb_cam输出日志

hhp_1.0.6RC1 (2022-09-09)
------------------
1. 增加launch文件中的参数：相机标定文件读取路径

hhp_1.0.6 (2022-08-30)
------------------
1. 新增读取相机标定文件并发布相机内参的功能

