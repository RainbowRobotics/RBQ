#!/bin/bash

APP_NAME="mediamtx"
APP_PATH="resources/mediamtx"

echo -ne "\033]0;$APP_NAME\007"

if pgrep -x "$APP_NAME" > /dev/null; then
    echo "$APP_NAME is already running. Please close it before starting a new instance."
    sleep 10
    exit 1
fi

if [ ! -f "$APP_PATH/$APP_NAME" ]; then
    echo "$APP_NAME application not found."
    sleep 10
    exit 1
fi

cd $APP_PATH

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        ./"$APP_NAME" 
    fi
    sleep 2
done
