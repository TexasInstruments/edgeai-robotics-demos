#!/bin/bash
DOCKER_TAG=pc-ros-noetic_scuttle:8.2
DOCKER_DIR=$HOME/j7ros_home/ros_ws/src/edgeai-robotics-demos/docker
IP_ADDR=$(ifconfig | grep -A 1 'eth0' | tail -1 | awk '{print $2}')
if [[ ! $IP_ADDR =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    IP_ADDR=$(ifconfig | grep -A 1 'wlp1s0' | tail -1 | awk '{print $2}')
fi
if [ "$#" -lt 1 ]; then
    CMD=/bin/bash
else
    CMD="$@"
fi
# modify the server and proxy URLs as requied
ping bitbucket.itg.ti.com -c 1 > /dev/null 2>&1
if [ "$?" -eq "0" ]; then
    USE_PROXY=1
else
    USE_PROXY=0
fi
xhost +local:$$USER
docker run -it --rm \
    -v $HOME/j7ros_home:/root/j7ros_home \
    -v /dev:/dev \
    --privileged \
    --network host \
    --env USE_PROXY=$USE_PROXY \
    --env J7_IP_ADDR=$J7_IP_ADDR \
    --env PC_IP_ADDR=$PC_IP_ADDR \
    --env='DISPLAY' \
    --env='QT_X11_NO_MITSHM=1' \
    --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \
      $DOCKER_TAG $CMD
xhost -local:$$USER
