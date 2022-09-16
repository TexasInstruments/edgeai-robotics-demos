# edgeai-robotics-demos - A Linux based project for running Robotics demos on TI platform

This edgeai-robotics-demos package maintains demos built under Python/CPP and are targeted for the
SCUTTLE platform. This document walks through what is contained in this repository, how to
configure the system and run the provided sample applications, and the scuttle library API.

# Package Components

In addition to this document, the library package contains the following:

1. The `python/common` subdirectory contains the Python modules that implement all
core library functionality.

2. The `python/samples` subdirectory contains sample applications to help in getting
familiar with the library API and getting started on an application. The
`drive_with_gamepad.py` application show how to use the high-level API from the
SCUTTLE and GamePad classes to drive the robot manually.

3. The `python/apps` sub-directory contains more complex applications demonstrating the
integration of the computation logic that leads to the commands driving the robot.

4. The `ros1` sub-directory contains the ROS version of the applications under `python/apps`
   sub-directory and a navigation application. Please see [Setting up and running demos under ROS](#setting-up-and-running-demos-under-ros) for details.

**NOTE: The file `python/common/scuttlepy/config/scuttle_sk_config.yaml` is a very important
configuration file that specifies the GPIO pin configuration for the motors and the
encoders on the SCUTTLE. In addition, it also has other important operational parameters.
The users should not modify this file unless there is a need to update some of the settings
that they understand.**

# Environment setup

Please follow the instructions in the Robotics SDK User Guide Documentation on [Setting up Robotics SDK](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_02_00/docs/source/docker/README.html#setting-up-robotics-kit-environment) to install Robotics SDK on J7 SK board. 

Once logged on to the SK board, clone the edgeai-robotics-demos repository.

``` shell
root@tda4vm-sk:/opt/edge_ai_apps# mkdir -p /opt/robot && cd /opt/robot
root@tda4vm-sk:/opt/robot# git clone -b EDGEAI_ROBOTICS_DEMOS_08.02.00.00 --single-branch https://github.com/TexasInstruments/edgeai-robotics-demos.git
```

Then add the repository path to the PYTHONPATH.

``` shell
root@tda4vm-sk:/opt/robot# export PYTHONPATH=${PYTHONPATH}:/opt/robot/edgeai-robotics-demos/python
```

## Install pre-requisites

Several packages need to be installed for running the code under this repository. Install the pre-requisites by running the following script.

```shell
root@tda4vm-sk:/opt/robot# edgeai-robotics-demos/scripts/setup_script.sh
```

# Running the sample scripts

**NOTE: Before running the demos on the SK board, the GPIO header should be enabled. Please
refer to the EdgeAI SK user guide on [the procedure for enabling the GPIO header](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_02_00/exports/docs/pi_hdr_programming.html).**

The following describes the operation of each application under the `samples/` directory:

1. `drive_with_gamepad.py`: This application uses the GamePad and SCUTTLE classes
to put together a simple chain for controlling the robot movement using the
gamepad controller. Only the left joystick is enabled. Use the joystick to
drive the robot in a desired direction.

Here are the steps to run the demo:

* Add the library path to PYTHONPATH environment variable
* Connect the gamepad USB dongle to one of the USB ports on the SK board
* Link the controller by pressing the HOME button on the controller. Make
sure to keep pressing it until the blue lights on the second and third
quadrant are lit.

To run these sample application. The state machine for the controller is captured in the link below.

[Common modules: Gamepad controller](python/common/README.md)

```shell
root@tda4vm-sk:/opt/robot# python3 edgeai-robotics-demos/python/samples/drive_with_gamepad.py
```

# Running the demos/applications

The following out-of-box demo applications are available under the `apps/` directory:

[OpenCV based ball follower](python/apps/opencv_subject_follower/README.md)

[EdgeAI based ball follower](python/apps/edgeai_subject_follower/README.md)

# Setting up and running demos under ROS

Please follow the link below for details on setting up and running the demos under ROS environment.

[Running the demos under Robotics SDK Docker based ROS environment](ros1/README.md)

# Warning

Any demo that uses the GPIO programming must ensure that the GPIOs used in a specifc application
are cleaned up properly at the end of the demo, especially the ones driving the PWM to the motor
controller. If these are left in a high state then the motors keep running and it might lead to
either cutting the power to the motors or power cycling the SK board.

# Documentation

Documentation on the SCUTTLE robot can be found at the following link:

* [SCUTTLE ROBOT DOCUMENTATION](https://scuttlerobot.org/resources)

Familiarize yourself with ``SCUTTLE WiringGuide.pdf`` content since the wiring on the robot with the
SK board is identical to the one that uses Raspberry Pi board.

Documentation on the scuttlepy library can be found at the following link:

* [SCUTTLEPY LIBRARY](https://github.com/ansarid/scuttlepy.git)

