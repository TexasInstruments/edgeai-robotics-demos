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

"""Motion Command Message

This script defines a message format fot the motion command used by the
robot control thread.

The message comprises of the identity of
* the sender (usually a thread running as a part fo the controlling application) and 
* the command, which is a tuple consisting of linear and angulat velocity components.

Example:
    # Create a message with explicit command and taskId specification
    msg1 = MotionCommand(cmd=(1.0, -0.3), tskId=ID_CONTROLLER_TASK)

    # Create a message with explicit command but default (ID_USER_TASK) taskId
    msg2 = MotionCommand(cmd=(1.0, -0.3))

    # Create a message with default command (0,0) and default (ID_USER_TASK) taskId
    msg3 = MotionCommand()

The taskId ID_CONTROLLER_TASK is reserved to be used by a game controller thread. The
design expects atmost two controllingg entities driving the robot, an optional game
controller and a user specific task (ex:- navigation stack or a ball following demo).
"""

# Id assigned to a user application task
ID_USER_TASK       = 10

# Id assigned to the gamepad controller task
ID_CONTROLLER_TASK = 11

class MotionCommand:
    """
    A class for defining a motion command to be used by the ScuttleDrive interface.

    Attributes:
        id (int): Id of the sender
        cmd (tuple): A tuple with the linear and angular velocities respectively.

    """
    def __init__(self, cmd=(0,0), tskId=ID_USER_TASK):
        """
        Constructor.

        Args:
            cmd: tuple
                The first one gives the linear velocity in m/s and the second one gives
                angular velocity in rad/sec

            taskId: integer
                Identity of the sender
        """
        self.id = tskId
        self.cmd = cmd

