# Running the demos under Robotics SDK Docker based ROS environment

This sections covers the details on setting up the ROS environment for running
the available demos under this repository.

## Setting Up Robotics SDK Docker Container Environment

The first step towards running the demos under ROS is to setup the Docker environment.

### Setting up J7 SK Board

Please follow the instructions in the Robotics SDK User Guide Documentation on [Docker Setup for ROS1](hhttps://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_02_00/docs/source/docker/setting_docker_ros1.html) to work with ROS1 on J7 SK board.

Once the robotics SDK docker container is set up, follow the instructions below to build a docker container for running the demos:

```shell
root@tda4vm-sk:/opt/robot# edgeai-robotics-demos/docker/build.sh
```

If the above commands are successful, you should have a docker image built successfully. An example output based on 8.2 release is shown below.

```shell
root@tda4vm-sk:/opt/robot# docker images

REPOSITORY              TAG                 IMAGE ID            CREATED             SIZE
j7-ros-noetic_scuttle   8.2                 a7b4a5e3fcdb        About an hour ago   3.25GB
j7-ros2-foxy            8.2                 50c5af3ef3c2        5 days ago          1.8GB
j7-ros-noetic           8.2                 71bb852e17e8        5 days ago          3.15GB
arm64v8/ubuntu          20.04               b5e8d9885aa9        6 days ago          65.6MB
```

### Setting up UBUNTU PC

Please follow the instructions in the Robotics SDK User Guide Documentation on [Set Up Docker Environment on the Remote PC for Visualization](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_02_00/docs/source/docker/setting_docker_ros1.html#set-up-docker-environment-on-the-remote-pc-for-visualization).

Once the robotics SDK docker container is setup on PC, follow the instructions below to clone the edgeai-robotics-demos repository and SCUTTLE_ROS:

```shell
user@pc:~/j7ros_home$ cd ros_ws/src
user@pc:~/j7ros_home/ros_ws/src$ git clone https://github.com/TexasInstruments/edgeai-robotics-demos.git
user@pc:~/j7ros_home/ros_ws/src$ git clone -b noetic https://github.com/scuttlerobot/SCUTTLE_ROS/ scuttle_ws
```

Then build a docker container for running the demos.

```shell
user@pc:~/j7ros_home/ros_ws/src$ edgeai-robotics-demos/docker/build_pc.sh
```

If the above commands are successful then you should have a docker image built successfully. An example output based on 8.2 release is shown below.

```shell
user@pc:~/j7ros_home/ros_ws/src$ docker images

REPOSITORY                    TAG       IMAGE ID       CREATED             SIZE
pc-ros-noetic_scuttle         8.2       f60c6c407314   About an hour ago   3.22GB
pc-ros-noetic                 8.2       4dfd0b15c047   3 days ago          3.19GB
ubuntu                        20.04     ba6acccedd29   3 days ago          72.8MB
```

## Building the demos

Log on to the Docker container on the SK board by running the following command.

``` shell
root@tda4vm-sk:/opt/robot# edgeai-robotics-demos/docker/run.sh
```

Follow the commands below to build the demos.

```shell
root@j7-docker:~/j7ros_home/ros_ws$ mkdir -p ros1_build
root@j7-docker:~/j7ros_home/ros_ws$ cd ros1_build
root@j7-docker:~/j7ros_home/ros_ws/ros1_build$ catkin_make --source /opt/robot
root@j7-docker:~/j7ros_home/ros_ws/ros1_build$ source devel/setup.bash
```

## Running the demos/applications
The following out-of-box demo applications are available under ROS1.

### OpenCV based ball follower

The details of this demo could be found under [OpenCV based ball follower](../python/apps/opencv_subject_follower/README.md).

**[J7 SK BOARD]** To launch the openCV ball follower demo, run the following:

```shell
root@j7-docker:~/j7ros_home/ros_ws/ros1_build$ source devel/setup.bash
root@j7-docker:~/j7ros_home/ros_ws/ros1_build$ roslaunch opencv_subject_follower opencv_follower.launch
```

**[Visualization on Ubuntu PC]** Run the following commands from within the ROS1 docker for visualizing the output.

```shell
user@pc:~/j7ros_home$ ros_ws/src/edgeai-robotics-demos/docker/run_pc.sh
root@pc-docker:~/j7ros_home/ros_ws$ rviz -d /root/j7ros_home/ros_ws/src/edgeai-robotics-demos/ros1/rviz/follower.rviz
```

**Note that `PC_IP_ADDR` and `J7_IP_ADDR` should be set before running Docker container on the ubuntu PC. Update these variables in `setup_env_pc.sh` and source it by the following command:**

```shell
user@pc:~/j7ros_home$ source setup_env_pc.sh
```

### EdgeAI based ball follower

The details of this demo could be found under [EdgeAI based ball follower](../python/apps/edgeai_subject_follower/README.md).

**[J7 SK BOARD]** To launch the openCV ball follower demo, run the following:

```shell
root@j7-docker:~/j7ros_home/ros_ws/ros1_build$ source devel/setup.bash
root@j7-docker:~/j7ros_home/ros_ws/ros1_build$ roslaunch edgeai_subject_follower edgeai_follower.launch
```

**[Visualization on Ubuntu PC]** Run the following commands from within the ROS1 docker for visualizing the output.

```shell
user@pc:~/j7ros_home$ ros_ws/src/edgeai-robotics-demos/docker/run_pc.sh
root@pc-docker:~/j7ros_home/ros_ws$ rviz -d /root/j7ros_home/ros_ws/src/edgeai-robotics-demos/ros1/rviz/follower.rviz
```

### Lidar based navigation

The details of this demo could be found under [Lidar based Navigation](./nodes/lidar_navigation/README.md).

**[J7 SK BOARD]** To launch the Lidar based navigation demo, run the following:

```shell
root@j7-docker:~/j7ros_home/ros_ws/ros1_build$ source devel/setup.bash
root@j7-docker:~/j7ros_home/ros_ws/ros1_build$ roslaunch lidar_navigation lidar_navigation.launch
```

**[Visualization on Ubuntu PC]** Run the following commands from within the ROS1 docker for visualizing the output.

```shell
user@pc:~/j7ros_home$ ros_ws/src/edgeai-robotics-demos/docker/run_pc.sh
root@pc-docker:~/j7ros_home/ros_ws$ rviz -d /root/j7ros_home/ros_ws/src/edgeai-robotics-demos/ros1/rviz/lidar_navigation.rviz
```

### Scuttlebot simulator on Gazebo

The details of this demo could be found under [scuttlebot_simulator package](./nodes/scuttlebot_simulator/README.md).


## Documentation

Documentation on the SCUTTLE ROS library can be found at the following link:

* [SCUTTLE ROS LIBRARY](https://github.com/scuttlerobot/SCUTTLE_ROS.git)

