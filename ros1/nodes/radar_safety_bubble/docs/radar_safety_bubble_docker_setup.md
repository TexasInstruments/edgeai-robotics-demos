Docker Setup for Radar Safety Bubble
====================================

This section covers the details on setting up the ROS environment for running the radar safety bubble demo.

**Note**: The Robotics SDK ROS1 Docker image must be built for both the TDA4 and the remote Ubuntu PC prior to building and running the Docker image for this demo. If you have not done so already, see the intructions to do so [here](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_02_00/docs/source/docker/setting_docker_ros1.html#)

# Setting up the J7 SK Board

Follow the instructions below to build a docker container for running the radar safety bubble demo:

```
root@tda4vm-sk:/opt/robot# radar-safety-bubble/docker/build.sh
```

If the above commands are successful, you should have a docker image built successfully. An example output based on 8.2 release is shown below.

```
root@tda4vm-sk:/opt/robot# docker images

REPOSITORY                  TAG             IMAGE ID         CREATED            SIZE
j7-ros-noetic_safety_bubble 8.2             a7b4a5e3fcdb     About an hour ago  3.25 GB
j7-ros2-foxy                8.2             50c5af3ef3c2     5 days ago         1.8GB
j7-ros-noetic               8.2             71bb852e17e8     5 days ago         3.15GB
arm64v8/ubuntu              20.04           b5e8d9885aa9     6 days ago         65.6MB
```

# Setting up UBUNTU PC

Build the docker container for running the radar safety bubble demo:

```
user@pc:~/j7ros_home/safety_bubble_ws/src$ radar-safety-bubble/docker/build_pc.sh
```

If the above commands are successful then you should have a docker image built successfully. An example output base on 8.2 release is shown below.

```
user@pc:~/j7ros_home/safety_bubble_ws/src$ docker images

REPOSITORY                  TAG             IMAGE ID         CREATED            SIZE
pc-ros-noetic_safety_bubble 8.2             a7b4a5e3fcdb     About an hour ago  3.22 GB
pc-ros-noetic               8.2             71bb852e17e8     5 days ago         3.19GB
ubuntu                      20.04           b5e8d9885aa9     6 days ago         72.8MB
```