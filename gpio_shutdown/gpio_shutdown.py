#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import os

def signal_handler(signal, frame):
    GPIO.cleanup()
    sys.exit(0)

# 定义使用的GPIO通道
shutdown_pin = 38  # BOARD 编码 38

# 检测到 38 号引脚的下降沿时关闭系统
def shutdown_system(channel):
    print("Shutdown button pressed! Shutting down...")
    os.system("sudo sync && sudo shutdown now")

def main():
    # Pin Setup
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)  # 使用 BOARD 编码方式
    GPIO.setup(shutdown_pin, GPIO.IN)  # 将 38 号引脚设置为输入

    # 注册按钮下降沿事件的中断处理函数
    GPIO.add_event_detect(shutdown_pin, GPIO.FALLING, callback=shutdown_system, bouncetime=10)

    print("Monitoring shutdown button (press CTRL+C to exit)")
    try:
        # 程序保持运行等待事件触发
        signal.pause()
    finally:
        GPIO.cleanup()  # 清理所有 GPIO

if __name__ == '__main__':
    # 捕捉 CTRL+C 信号
    signal.signal(signal.SIGINT, signal_handler)
    main()
