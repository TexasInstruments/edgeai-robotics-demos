#!/bin/bash
DOCKER_TAG=j7-ros-noetic_scuttle:8.2
DOCKER_DIR=/opt/robot/edgeai-robotics-demos/docker
# modify the server and proxy URLs as requied
ping bitbucket.itg.ti.com -c 1 > /dev/null 2>&1
if [ "$?" -eq "0" ]; then
    USE_PROXY=1
    REPO_LOCATION=artifactory.itg.ti.com/docker-public-arm
    HTTP_PROXY=http://webproxy.ext.ti.com:80
else
    USE_PROXY=0
    REPO_LOCATION=arm64v8
fi
echo "USE_PROXY = $USE_PROXY"
echo "REPO_LOCATION = $REPO_LOCATION"
docker build \
    -t $DOCKER_TAG \
    --build-arg USE_PROXY=$USE_PROXY \
    --build-arg REPO_LOCATION=$REPO_LOCATION \
    --build-arg HTTP_PROXY=$HTTP_PROXY \
    -f $DOCKER_DIR/Dockerfile.arm64v8.noetic .

