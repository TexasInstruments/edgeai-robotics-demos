EdgeAI based subject follower
=============================

# Demo details

## Goal

The goal of the demo is to demonstrate the following key aspects:

1. Sense the scene using either a mono or stereo camera (ZED1 or ZED2). Even when using a stereo camera, either the left or right image will be used.
2. Run the Deep Learning based 2D object detection chain. The output of this chain will be a list of bounding boxes with labels and confidence measures.
3. Implement a simple logic to post-process the 2D bounding box data and steer the SCUTTLE robot in the direction of the object of interest.

The robot shall operate with the following constraints placed on the environment:
1. The demonstration is restricted to indoor environment.
2. The ground plane is expected to be flat and free of any bumps.
3. Cameras placed at a sufficient height. This needs to be determined based on some initial runs. The idea is to understand the sweet spot where a given OD model is going to perform with sufficient accuracy.

## Demo Description
In this demo, the robot follows a specific object. The following diagrams depicts the processing flow of this demo application:

<p align="center">
    <img src="../../../docs/images/scuttle_edgeai_demo1_process_flow.svg" width="900">
</p>

<p align="center"><b>Figure 1. EdgeAI Object Detection based Subject following</b></p>

Refer to the link below on details of the subject following logic.

[Common modules: Subject follower](../../../python/common/README.md)

The edgeAI processing chain will consume the input camera image and outputs bounding boxes, if any objects are detected. The output also
consists of the object class information. The bounding box information is used by the follower logic to determine the position of the object
in the image plane. Since a mono camera is used in the demo, there is no depth information available and the width of the object is used
as an measure to decide if an object is far away or close to the robot. This is a very crude way of estimating the depth for the sake of tracking.

The parameters for controlling the above are specified in [python/apps/edgeai_subject_follower/subject_follower.yaml](subject_follower.yaml) configuration file.
The object being tracked is specified by the `target_class` parameter. This classId should match one of the valid classes output by the AI model used for
the object detection operation.

The configuration controlling the object detection AI chain is specified under `models:model0:model_path` parameter
in [python/apps/edgeai_subject_follower/process_config.yaml](process_config.yaml) file.

## Hardware and Software Setup

The following setup is necessary for this demo:

1. SCUTTLE robot with fully charged battery pack powering the onboard DC motors.
2. A working J721e SK board, E2 revision of higher, securely mounted on the SCUTTLE platform.
   1. Appropriate GPIO connections from the SK board to the SCUTTLE motor control board.
   2. The SD card flashed with the latest stable EdgeAI SDK with Robotics SDK optionally setup (needed only if using ROS based demos).
   3. Demo code compiled and ready to run on the SK board.
3. Fully charged power supply for powering the SK board and optionally the ZED stereo camera, if one is mounted.
4. A mono or stereo camera secured to the front of the SCUTTLE platform and connected to USB3 (USB-C port to be specific if ZED stereo camera) on the SK board.
5. A compatible WiFi card with antennae installed on the SK board. The antennae must be secured so that no accidental damage to either the WiFi card, antennae, of the SCUTTLE robot is done.
6. Game controller USB dongle connected to one of the the USB ports on the SK board

### Running the demo
It is expected that all the necessary installation steps outlined on the main page have been followed and all the dependent libraries are installed and the PYTHONPATH has been setup.

The following steps will launch the demo:
1. Power up the robot with a WiFi card installed
2. Connect the laptop to the SK board. Scan for the Access Point that is tied to the SK board and connect.
3. SSH into the board.
4. cd <path_to_edgeai-robotics-demos>
5. Turn on the power to the motors.

Run the following commands.

```shell
# Add the edge_ai python library path to the PYTHONPATH
root@tda4vm-sk:~# export PYTHONPATH=/opt/edge_ai_apps/apps_python:${PYTHONPATH}

# Run the demo
root@tda4vm-sk:~# python3 /opt/robot/edgeai-robotics-demos/python/apps/edgeai_subject_follower/edgeai_subject_follower.py /opt/robot/edgeai-robotics-demos/python/apps/edgeai_subject_follower/subject_follower.yaml
```
The user can take over the robot controller, if a gamepad is connected to the system. The idea behind the user control is
to override the robot motion, when necessary. The button assignment on the gamepad controller is as follows. The gamepad
button events are always processed and acted upon. The state machine for the controller is captured in the link below.

[Common modules: Gamepad motor controller](../../../python/common/README.md)

If no gamepad is connected then the robot will be put in a state to respond to the ball tracking algorithm upon the demo launch.

Once the robot is put in a state to respond to the ball tracking algorithm, the robot will be ready to move. The app has been programmed to track a 
subject with classId programmed in the YAML file.

Get a basket ball into the FOV of the camera and the robot should try to orient itself towards the ball. The robot will
follow the ball, if the ball is moved around with a slow speed.

To kill the demo, hit `CTRL-C` on the terminal and the motors should stop. Switch off the power supply to the motors to
conserve the battery.

