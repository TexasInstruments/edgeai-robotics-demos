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

"""OpenCV based subject follower

This script implements the opencv  based subject follower application.
"""

import sys
import cv2
import numpy as np
import time
import yaml

from common.logger import *
from common.command_line_parse import get_cmdline_args
from common.subject_follower import SubjectFollower
from common.gamepad_motor_control import *
from common.robot_drive import *

# A function to capture an image & return all pixel data
def newImage(camera, size):
    ret, image = camera.read()          # return the image as well as ret
    if not ret:                         # (ret is a boolean for returned successfully?)
        print("NO IMAGE")
        return None                     # (return an empty var if the image could not be captured)

    # Resize the image to reduce processing overhead
    image = cv2.resize(image, size, interpolation=cv2.INTER_LINEAR)
    return image

def colorTarget(image, color_range): # function defaults to open range if no range is provided
    # The input image is in BGR format. Convert to HSV
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply the threshold to filter the image. The pixels outside the range will be black
    # after this operation.
    thresh = cv2.inRange(image_hsv, color_range[0], color_range[1])

    # Perform guassian blur. The following does the Dilation followed by Erosion.
    # It closes any holes inside the object.
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Find all contours
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, 
                                cv2.CHAIN_APPROX_SIMPLE)[-2]    # generates number of contiguous "1" pixels

    targ = None
    if len(contours) > 0:                                       # begin processing if there are "1" pixels discovered
        c = max(contours, key=cv2.contourArea)                  # return the largest target area
        #x,y,w,h = cv2.boundingRect(c)
        #radius = w/2

        ((x, y), radius) = cv2.minEnclosingCircle(c)            # get properties of circle around shape

        # Find the centroid of the blob. The coordinates of the
        # centroid can be computed as follows:
        # C_x = M10/M00
        # C_y = M01/M00
        M = cv2.moments(c)

        if M["m00"]:
            x,y = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        targ = [x, y, radius]

    # return x, y, radius, of target
    return targ

class Follower:
    """
    A class for providing the interface to a robot under control. 

    The details of the detected object are passed to the subject follower
    algorithm instance for deriving the robot motion control commands and
    sending them over to the robot control interface.

    """
    def __init__(self, config, logger, gamepad_control=None, robot_instance=None):
        self.thread       = None
        self._stop_thread = False
        self.cb           = None
        self.logger       = logger

        # Print the demo title
        self.logger.info(config['title'])

        fps              = config['framerate']
        input_width      = config['input_width']
        input_height     = config['input_height']
        min_radius       = config['min_radius']
        target_radius    = config['target_radius']
        center_threshold = config['center_threshold']
        hsv_min          = config['hsv_min']
        hsv_max          = config['hsv_max']

        camera = cv2.VideoCapture(config['source'])

        if camera.isOpened() == False:
            self.logger.error("Error opening camera.")
            sys.exit(1)

        camera.set(cv2.CAP_PROP_FPS, fps)
        fps_read = camera.get(cv2.CAP_PROP_FPS)

        if fps_read != fps:
            self.logger.error("Failed to ser fps to {}".format(fps))
            camera.release()
            sys.exit(1)
        else:
            self.logger.info("FPS: {}".format(fps))

        self.fps         = fps
        self.camera      = camera
        self.size        = (input_width, input_height)
        self.color_range = ((hsv_min[0], hsv_min[1], hsv_min[2]),
                           (hsv_max[0], hsv_max[1], hsv_max[2]))
        self.follower    = SubjectFollower(input_width=input_width,
                                           input_height=input_height,
                                           min_radius=min_radius,
                                           target_radius=target_radius,
                                           center_threshold=center_threshold)

        if robot_instance:
            self.robot = robot_instance
            # If there is no gamepad controller connected then take the robot
            # out of the emergency stop state
            if gamepad_control == None:
                self.robot.register_control_task(ID_USER_TASK)

    def registerCallback(self, cb):
        self.cb = cb

    def callback(self, data):
        image, radius, chassisTargets = data

        cmd = MotionCommand(chassisTargets)
        self.robot.sendCommand(cmd)

    def _proc_thread(self):
        try:
            while (self._stop_thread == False) and self.camera.isOpened():
                image = newImage(self.camera, self.size)

                if image is None:
                    raise Exception("Image capture failed.\n")
                    
                # Locate a target
                target = colorTarget(image, self.color_range)
                if target != None:
                    x, y, radius = target
                    chassisTargets = self.follower.getChassisTargets(x, y, radius)
                    self.logger.info("X = {0:.2f} Y = {1:.2f} R:{2:.2f} T:{3}".\
                                     format(x, y, radius, chassisTargets))
                else:
                    radius = 0;
                    chassisTargets = [0.0, -0.5]

                self.cb((image, radius, chassisTargets))

                time.sleep(1.0/self.fps)
        except KeyboardInterrupt:
            if self.camera.isOpened():
                self.camera.release()

    def start(self):
        if self.thread == None:
            self.thread = threading.Thread(target=self._proc_thread)
            self.thread.start()

    def wait_for_exit(self):
        self.thread.join()

        # Release the instance of the robot control object
        release_robot_control_instance()
        self.thread = None

    def stop(self):
        if self.thread != None:
            self._stop_thread = True
            self.wait_for_exit()

def main(sys_argv):
    args = get_cmdline_args(sys_argv)

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    # Create a logger object
    logger = create_logger(name='opencv_subject_follower', level=args.log_level)

    # If the gamepad is not connected, issue a warning and proceed
    try:
        # Launch the gamepad controller thread
        gamepad_control = GamepadMotorControl(config=config['robot_config'])
        gamepad_control.start()
    except Exception as e:
        print("WARNING:", e)
        gamepad_control = None

    try:
        robot_instance = get_robot_control_instance(config['robot_config'])
        follower = Follower(config, logger, gamepad_control, robot_instance)
        follower.registerCallback(follower.callback)
        follower.start()
        follower.wait_for_exit()
    except KeyboardInterrupt:
        follower.stop()

    if gamepad_control:
        gamepad_control.stop()

if __name__ == "__main__":
    main(sys.argv)
