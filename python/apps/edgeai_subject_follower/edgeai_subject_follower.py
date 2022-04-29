#!/usr/bin/python3
#  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

""" EdgeAI subject follower

This script implements the EdgeAI processing chain based subject follower application.

"""

import sys
import yaml

from common.logger import *
from common.command_line_parse import get_cmdline_args
from common.subject_follower import SubjectFollower
from common.gamepad_motor_control import *
from common.robot_drive import *

from follower_demo_class import FollowerDemo

class CallbackObj:
    """
    A class for providing the interface to a robot under control. An object
    of this type is registered with the EdgeAI processing chain such that
    once the data has been through the inference stage, the post-processed
    image and the associated detection information is passed to this object
    for further action.

    The details of the detected object are passed to the subject follower
    algorithm instance for deriving the robot motion control commands and
    sending them over to the robot control interface.

    """

    def __init__(self, config, gamepad_control):
        self.thresh        = config['prob_threshold']
        self.target_class  = config['target_class']
        robot_config       = config['robot_config']
        log_level          = config['log_level']
        min_radius         = config['min_radius']
        target_radius      = config['target_radius']
        center_threshold   = config['center_threshold']
        self.input_width   = config['input_width']
        self.input_height  = config['input_height']

        self.logger = create_logger(name='edgeai_subject_follower',
                                    level=log_level)

        try:
            self.follower = SubjectFollower(input_width=self.input_width,
                                          input_height=self.input_height,
                                          min_radius=min_radius,
                                          target_radius=target_radius,
                                          center_threshold=center_threshold)
            self.robot = get_robot_control_instance(robot_config)

            # If there is no gamepad controller connected then take the robot
            # out of the emergency stop state
            if gamepad_control == None:
                self.robot.register_control_task(ID_USER_TASK)
        except Exception as e:
            print("Exception: {0}".format(e))
            sys.exit(2)

    def __call__(self, img, box, class_name, reset=False):
        """
        A functor invoked by the tailend of the EdgeAI processing chain.

        Args:
            img (image): Post-processed image from the EdgeAI object detection chain
            box (list): A list of 4 values representing the top-left and bottom-right
                        co-ordinates of the bounding box surrounding a detected object.
            class_name (str): A string representation of the class of the detected object
            reset (bool): A flag to indicate if the detection data should be looked at.
        """

        chassisTargets = [0, -0.5]

        if reset == False:
            # Compute the center (x,y) of the object and derive the width.
            x      = (box[0]+box[2])/2
            y      = (box[1]+box[3])/2
            radius = (box[2]-box[0])/2

            # Obtain the linear and angular valocity components
            chassisTargets = self.follower.getChassisTargets(x, y, radius)
            self.logger.info("X = {0:.2f} Y = {1:.2f} R:{2:.2f} T:{3}".format(x, y, radius, chassisTargets))
        else:
            self.logger.info("T:{}".format(chassisTargets))

        # Build the robot motion command message and send it to the robot control
        cmd = MotionCommand(chassisTargets)
        self.robot.sendCommand(cmd)

    def stop(self):
        # Release the robot object context
        release_robot_control_instance()

    def __del__(self):
        self.stop()

# class CallbackObj

def main(sys_argv):
    """
    Main function implementing the application.

    """

    args = get_cmdline_args(sys_argv)

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    config['log_level'] = args.log_level

    # Get the parameters for setting up the demo object
    with open(config['process_config'], 'r') as f:
        demo_config = yaml.safe_load(f)

    demoObj = None
    cbObj   = None

    # If the gamepad is not connected, issue a warning and proceed
    try:
        # Launch the gamepad controller thread
        gamepad_control = GamepadMotorControl(config=config['robot_config'])
        gamepad_control.start()
    except Exception as e:
        print("WARNING:", e)
        gamepad_control = None

    try:
        # Create a callback object and register it with the processing chain
        cbObj = CallbackObj(config, gamepad_control)
        demo_config['callback_object'] = cbObj

        # Create the demoObj object
        demoObj = FollowerDemo(demo_config)

        # Start the demoObj
        demoObj.start()

        demoObj.wait_for_exit()
    except KeyboardInterrupt:
        pass
    finally:
        pass

    if demoObj:
        demoObj.stop()
        del demoObj

    if cbObj:
        cbObj.stop()
        del cbObj

    if gamepad_control:
        gamepad_control.stop()

if __name__ == "__main__":
    main(sys.argv)
