#!/bin/bash

APP_NAME="rbq_driver"
echo -ne "\033]0;$APP_NAME\007"

while true; do
    echo "$APP_NAME is deprecated and embedded to core-sw remove start_ros_driver.bash from RBQ.bash"
    sleep 10
done
