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

"""Scuttle Drive

This script implements SCUTTLE specific control logic.

"""
import sys

from common.robot_if import *
from scuttlepy import *

class ScuttleDrive(RobotIf):
    """
    A class for controlling the interface to the robot motor control. This inherits
    from the RobotIf class and overrides a few methods for the SCUTTLE robot specific
    operation.

    """

    def __init__(self, config, queue_time_out=RobotIf.IN_QUEUE_TIMEOUT):
        """
        Args:
            config: str, optional
                Name of the YAML configuration file definining the robot operational parameters.
                If 'None' is specified then the following file will be used:
                /opt/robot/edgeai-robotics-demos/python/common/scuttlepy/config/scuttle_sk_config.yaml
            queue_time_out (int): Time-out for command queue reads

        """

        # Invoke the baseclass constructor
        RobotIf.__init__(self, config, queue_time_out)
        self.start()

    def start(self):
        """
        Starts the processing thread. The caller should call wait_for_exit() if
        the calling application has no other means blocking until this object is
        destroyed.

        Overwrites the base class start() method
        """
        if self.thread == None:
            self._robot = SCUTTLE(config=self._config, openLoop=True)
            self.thread = threading.Thread(target=self._control_thread)
            self.thread.start()

