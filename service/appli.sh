#!/bin/bash

HOME=/root
WS=$HOME/dev_ws/gz
export CAM_TYPE=usb

source /opt/tros/setup.bash
source $WS/install/setup.bash

ros2 launch gz car.launch.py

echo "Done! With ret code $?"

