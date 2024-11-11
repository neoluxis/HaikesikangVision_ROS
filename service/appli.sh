#!/bin/bash

me=$(readlink -f $0)
echo "Running $me"

if [ -L /usr/local/bin/appli.sh ]; then
    rm /usr/local/bin/appli.sh
fi
echo "Linking $me to /usr/local/bin/appli.sh"
ln -sf $me /usr/local/bin/appli.sh

HOME=/root
WS=$HOME/dev_ws/appli
export CAM_TYPE=usb

source /opt/tros/humble/setup.bash
source $WS/install/setup.bash

ros2 launch $WS/launch/run_all.launch.py

echo "Done! With ret code $?"

