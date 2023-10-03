#!/bin/bash

xhost +local:docker
docker run --rm \
    --gpus all \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -e 'QT_X11_NO_MITSHM=1' \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v `pwd`/apriltags_ws:/root/apriltags_ws \
    --ipc host \
    --device /dev/dri \
    --device /dev/snd \
    --device /dev/input \
    --device /dev/bus/usb \
    --privileged \
    --ulimit rtprio=99 \
    --net host \
    --name ubuntu_tags \
    --entrypoint /bin/bash \
    -ti docker_ros:apriltags

# maping the usb device directory from the machine to the container
# -v /dev/bus/usb/:/dev/bus/usb\    
