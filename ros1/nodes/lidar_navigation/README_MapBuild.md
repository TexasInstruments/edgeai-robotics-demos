How to Create a Map using a Lidar Sensor from SCUTTLE
====================================================

## Goal

In this document, we show how to build a map using a 360&deg; 2-D Lidar sensor, RPLidar A1, and Hector SLAM on the SK board. The map will be used in the navigation application. By creating the map, we can demonstrate the following key aspects:

1. Scan the scene using a 360&deg; 2-D Lidar sensor, RPLidar A1. 
2. Estimate the change of a robot's pose by fusing the data from a wheel encoder and an IMU sensor, BNO055. 
3. Refine the pose by matching the Lidar scan with the previous Lidar scans and build a 2-D occupancy grid map by accumulating them. 

The robot shall operate with the following constraints placed on the environment:
1. The demonstration is restricted to indoor environment.
2. The ground plane is expected to be flat and free of any bumps.
3. A RPLidar A1 shouldn't be blocked by any other sensors and structures on a robot where the Lidar sensor is mounted. Objects should be taller than the height of the Lidar sensor.

## Hardware and Software Setup

The following setup is necessary for this demo:

1. SCUTTLE robot with fully charged battery pack powering the onboard DC motors.
2. A working J721e SK board, E2 revision of higher, securely mounted on the SCUTTLE platform. 
   1. Appropriate GPIO connections from the SK board to the SCUTTLE motor control board.
   2. The SD card flashed with the latest stable EdgeAI SDK with Robotics SDK optionally setup. <br/>
   3. Demo code compiled and ready to run on the SK board.
3. Fully charged power supply for powering the SK board and a USB Hub, through which the Lidar sensor is connected to the SK board.
4. A RPLidar A1 sensor secured to the front of the SCUTTLE platform and connected to the USB port on the SK board through the USB Hub. 
5. A compatible WiFi card with antennae installed on the SK board. The antennae must be secured so that no accidental damage to either the WiFi card, antennae, of the SCUTTLE robot is done.
6. Game controller USB dongle connected to one of the USB ports on the SK board

## Building a Map

**[J7 SK BOARD]**

The Docker environment should be set up beforehand based on [Robotics SDK User Guide](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_02_00/docs/source/docker/README.html#setting-up-robotics-kit-environment). It is also expected that the pre-requisites have been installed as described in [edgeai-robotics-demos](../../../README.md) and the SCUTTLE_ROS package has been cloned as described in [Running the demos under Robotics SDK Docker based ROS environment](../../../ros1/README.md).
 
First start a Docker container and compile SCUTTLE_ROS in `/opt/robot`.

```shell
root@tda4vm-sk:~# j7ros_home/docker_run_ros1.sh
root@j7-docker:~/j7ros_home/ros_ws$ catkin_make --source /opt/robot
```

Extend the PYTHONPATH environment to use the robot control library.

``` shell
root@j7-docker:~/j7ros_home/ros_ws$ export PYTHONPATH=/opt/ti-gpio-py/lib/python:/root/opt/robot/scuttlepy:${PYTHONPATH}
```

Turn on the power to the motors and launch scuttle_bringup package and gamepad controller. It launches the RPLidar sensor and publishes wheel odometry information.

```shell
root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
root@j7-docker:~/j7ros_home/ros_ws$ roslaunch lidar_navigation lidar_mapping.launch
```

We move the SCUTTLE robot using the game controller around the area we like to build a map. Before that, we need to launch Hector SLAM on Ubuntu PC.

**[Ubuntu PC]**

It is expected that the Robotics SDK and the Docker environment have been set up on Ubuntu PC based on [Robotics SDK User Guide](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_02_00/docs/source/docker/README.html#setting-up-robotics-kit-environment). 

Launch the Docker container, and clone the Hector SLAM git repository and apply a patch. 

``` shell
user@pc:~/j7ros_home$ ./docker_run_ros1.sh
root@pc-docker:~/j7ros_home/ros_ws$ export SLAM_ROOT=$HOME/j7ros_home/ros_ws/src/robotics_sdk/ros1/slam
root@pc-docker:~/j7ros_home/ros_ws$ $SLAM_ROOT/j7_setup_hector_slam.sh
```

Then, compile and launch the Hector SLAM by the following commands.

``` shell
root@pc-docker:~/j7ros_home/ros_ws$ catkin_make --source ./src/robotics_sdk/ros1/slam/hector_slam
root@pc-docker:~/j7ros_home/ros_ws$ roslaunch hector_mapping scuttle_slam.launch
```

As the robot moves, we can see that the 2D occupancy grid map is updated. Once the map is successfully created, save the map using map_server. It saves the map into the pgm image format along with a YAML file. The YAML file and the pgm image should be in the same directory and the navigation demo can load it. 

```shell
root@pc-docker:~/j7ros_home/ros_ws$ rosrun map_server map_saver -f map_name
```

## Editing a Map
The created map may have some noises and errors in it. It is recommended to remove such noises and mapping errors using an image application such as gimp. For example, install and run gimp by the following commands.

```shell
user@pc:~/j7ros_home$ sudo apt-get update
user@pc:~/j7ros_home$ sudo apt-get install gimp
user@pc:~/j7ros_home$ gimp
```

Load the map image using **File->Open**, and save it after editing.
