Common utility modules
======================

# Description

The goal of the common utility code is to promote the re-use of the modules across different demos. The sections below explain each of the
re-usable modules in detail.

## Subject follower

This module implements the main logic for moving the robot towards the object of interest. It is a simple algorithm which takes
the location and size of an object to be followed as input and computes the linear and angular velocity components that need to
be sent to the motor control code. This code is located in `subject_follower.py` module.

The following table describes the configurable algorith parameters:

| Parameter | Description |
| :-------: | :---------: |
| input_width | Width of the input image |
| input_height | Height of the input image |
| min_radius | Minimum radius of the object to trigger computations |
| target_radius | This is the radius that the algorithm tries to converge to. |
| center_threshold | The threshold use for computing angular velocity component |

The following picture depicts a object following logic:

<p align="center">
    <img src="../../docs/images/scuttle_edgeai_demo1.svg" width="900">
</p>

<p align="center"><b>Figure 1. Ball Following Demo details</b></p>

The location of the object in the camera frame is used to compute the linear and angular velocities as follows. The angular correction will
be applied before applying the linear velocity component.
1. If the object width is greater than a threshold, then the object is considered to be closer and the linear velocity component
   will be negative so that the robot will back-off. The magnitude will be proportional to the separation of the robot from the object.
2. If the object width is less than a threshold, then the object is considered farther away from the robot and the linear velocity
   component will be positive so that the robot can approach the object. The magnitude will be proportional to the separation of
   the robot from the object.
3. If the object is not centered in the image plane, then an angular velocity component will be applied first so that the object is
   with in a center band with respect to the camera. A +ve value of the angular valocity steers the robot Counter Clockwise (CCW)
   and a -ve value steers it in the Clockwise (CW) direction.

## Scuttle drive

The scuttle_drive module provides a controlled interface to the robot motion control library. It has the following features:
1. Provides a messaging interface to multiple entities that want to control the robot motion. The following entities are supported
    1.1. The application that is running a demo. This will be identified with a pre-defined taskId ID_USER_TASK.
    1.2. The gamepad controller task. This is identofied with a pre-defined taskId ID_CONTROLLER_TASK.
    1.3. The scuttle_drive itself, which has the highest priority to act on the timeout event. The Id is internal.
2. Implements a timeout mechanism where by if no messages are received within the timeout period, the module stops the robot
   motion. The idea here is to make sure that in the event of control breakdown, the robot motion is halted to prevent any
   accidents.
3. Provides helper API, get_robot_control_instance() and release_robot_control_instance() to provide a singleton like behavior
   which ensures that there is atmost one instance of the robot control.

## Gamepad motor controller

The gamepad controller module can either be instantiated by itself to control the robot or used in conjunction with other control
modules that send move commands to the robot. This code is located in `gamepad_motor_control.py` module.

The user can take over the robot controller, if a gamepad is connected to the system. The idea behind the user control is
to override the robot motion, when necessary. 

The state machine for the controller is as shown below. It shows all the possible state transitions when run in conjunction with an
application that also controls the robot movements.

<p align="center">
    <img src="../../docs/images/scuttle_controller_state_machine.svg" width="900">
</p>

<p align="center"><b>Figure 2. Game Controller State Machine</b></p>

**NOTE**: Pressing buttons Y and START will make the motor controllable both from the external appliccation as well as the
          gamepad controller; while this can come handy, it should be avoided for a smooth control.
