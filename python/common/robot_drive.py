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

"""
This script provides a unified interface for getting robot context objects.

The idea is to isolate specific robot models from the application code. If a
specific robot needs to be used then the user need to modify the methods provided
in this script with the constraints that the robot object exposes standard API for
sending commands.

"""

from common.scuttle_drive import *

__all__ = ['get_robot_control_instance',
           'release_robot_control_instance']

_robot = None
"""Variable to hold the robot object reference"""

_instance_count = 0
"""Instance count for implementing reference counting """

def get_robot_control_instance(config):
    """
    Cretaes an instance of the ScuttleDrive class and returns to the caller. If the
    object has been already created then a reference to the object is returned. A
    reference count is incremented to keep track of the object use.

    Args
        config (str): A path to the robot configuration parameters in YAML format.

    Returns:
        A fully consructed robot context.

    """
    global _robot
    global _instance_count

    if _robot == None:
        _robot = ScuttleDrive(config)

    _instance_count = _instance_count + 1
    return _robot

def release_robot_control_instance():
    """
    Decrements the reference count of the global ScuttleDrive object and destroys the object
    once the reference count goes to 0.
    """
    global _robot
    global _instance_count

    if _instance_count > 0:
        _instance_count = _instance_count - 1
        if _instance_count == 0:
            _robot.stop()
            del _robot
            _robot = None
