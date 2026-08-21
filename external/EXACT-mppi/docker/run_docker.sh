#!/bin/bash

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
echo "Starting container. Mounting $REPO_ROOT to /workspace/EXACT-mppi"

xhost +local:root

docker run -it --rm --privileged --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${REPO_ROOT}:/workspace/EXACT-mppi:rw" \
    -w "/workspace/EXACT-mppi" \
    exact-mppi:humble-cuda \
    bash

xhost -local:root
