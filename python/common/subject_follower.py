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

"""Subject follower implementation

This script implements the subject follower logic as a class. The idea is to reuse the
implementation across openCV and EdgeAI based processing chains.

"""

__all__ = ['SubjectFollower']

def _clamp(val, minVal, maxVal):
    return max(min(val, maxVal), minVal)

class SubjectFollower:
    """
    Class implementing the subject follower logic.

    Attributes:
        maxLinVel (float): Maximum linear velocity in m/s
        maxAngVel (float): Maximum angular valocity in rad/s
        target_radius (int): Desired radius in pixels. This is the radius that the algorithm \
                             strives to achieve through motion.
        min_radius (int): Value in pixels which triggers the computation of the linear velocity
        input_width (int): Width of the image in pixels
        input_height (int): Height of the image in pixels
        center_threshold (int): A range specification in pixels which is used to determine \
                                the angular velocity component for turning. If the object \
                                being followed is within this threshold range, then the \
                                angular velocity will be 0.
        deadBand (int): An angular velocity threshold for avoid oscillations

    """

    maxLinVel = 0.4
    maxAngVel = 2

    def __init__(self, input_width, input_height, min_radius, target_radius, center_threshold=50):
        """
        Initializes the internal state of the object.

        Args:
            input_width (int): Width of the image in pixels
            input_height (int): Height of the image in pixels
            min_radius (int): Value in pixels which triggers the computation of the linear velocity
            target_radius (int): Desired radius in pixels. This is the radius that the algorithm \
                                 strives to achieve through motion.
            center_threshold (int): A range specification in pixels which is used to determine \
                                    the angular velocity component for turning. If the object \
                                    being followed is within this threshold range, then the \
                                    angular velocity will be 0.

        """

        self.target_radius    = abs(target_radius)
        self.min_radius       = abs(min_radius)
        self.input_width      = abs(input_width)
        self.input_height     = abs(input_height)
        self.center_threshold = center_threshold
        self.deadBand         = 0.3  # Angular valocity range to mute to remove oscillations when object is centered
        
        # Generate the parameters for computing angular velocity
        self.angVelSlope, self.angVelInter = \
        SubjectFollower._slope_intercept(0,
                                        SubjectFollower.maxAngVel,
                                        input_width,
                                        -1*SubjectFollower.maxAngVel)
        
        # Generate the parameters for computing linear velocity
        self.linVelSlope, self.linVelInter = \
        SubjectFollower._slope_intercept(input_height,
                                        -1*SubjectFollower.maxLinVel,
                                        target_radius,
                                        0)

    def _turnAndGo(self, x, y, radius):
        """
        Computes the linear and angular valocity components to be applied to align
        with an object centered at (x,y) with radius/width 'radius'.

        Args:
            x (int): X-coordinate of the pixes representing the center of the object in the image plane
            y (int): Y-coordinate of the pixes representing the center of the object in the image plane
            radius (float): Radius or width of the object in pixels

        Returns:
            An array with two elements, the first one representing the linear velocity component and
            the second one representing the angular velocity component.

            If any of the arguments are None, then a default [0.0, -0.2] is returned. This corresponds
            to the clockwise rotation of 0.2 rad/s. This is typically the case where no object has been
            detected and the robot needs to scan in a partocular direction. The direction chosen here is
            arbitrary.

        """

        chassisTargets = [0.0, -0.2]

        if x is None or y is None or radius is None:
            return chassisTargets

        pos = self.input_width/2 - x

        if abs(pos) > self.center_threshold:
            angVel = (self.angVelSlope * x) + self.angVelInter
            angVel = round(_clamp(angVel, -1*SubjectFollower.maxAngVel, SubjectFollower.maxAngVel), 2)
            if abs(angVel) < self.deadBand:
                angVel = 0
            chassisTargets[1] = angVel
        if radius > self.min_radius:
            linVel = (self.linVelSlope * radius * 2) + self.linVelInter
            linVel = round(_clamp(linVel, -1*SubjectFollower.maxLinVel, SubjectFollower.maxLinVel), 2)
            chassisTargets[0] = linVel

        return chassisTargets

    @classmethod
    def _slope_intercept(self, x1, y1, x2, y2):
        """
        Derives the slope and intercept of a line passing through (x1, y1) and (x2, y2).

        This is used to linearly interpolate the linear and angular velocities for tracking
        the objects in the image plane.

        """
        a = (y2 - y1) / (x2 - x1)
        b = y1 - (a * x1)
        return a,b

    def getChassisTargets(self, x, y, radius):
        """
        A public function for returning the linear and angular valocity components to
        follow a target object with a center at (x, y) and with radius 'radius'.

        Args:
            x (int): X-coordinate of the pixes representing the center of the object in the image plane
            y (int): Y-coordinate of the pixes representing the center of the object in the image plane
            radius (float): Radius or width of the object in pixels

        Returns:
            An array with two elements, the first one representing the linear velocity component and
            the second one representing the angular velocity component.

            If any of the arguments are None, then a default [0.0, -0.2] is returned. This corresponds
            to the clockwise rotation of 0.2 rad/s. This is typically the case where no object has been
            detected and the robot needs to scan in a partocular direction. The direction chosen here is
            arbitrary.

        """
        chassisTargets = self._turnAndGo(x, y, radius)
        return chassisTargets

# class SubjectFollower

