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

"""Gamepad controller for motor control

This script provides a class for interfacing with a gamepad controller
and mapping a few button events to generate robot motion commands. The
interface provided by the robot_motion_command is used to form the robot
control messages.
"""

import sys
import time
import threading

# Import Internal Programs
import common.gamepad as gp
from common.robot_motion_command import *
from common.robot_drive import *

BUTTON_Y     = 4
"""int: Button for enabling gamepad controller """

BUTTON_X     = 7
"""int: Button for disabling gamepad controller """

BUTTON_B     = 5
"""int: Button for setting emergency stop flag in the robot context """

BUTTON_A     = 6
"""int: Button for resetting emergency stop flag in the robot context """

BUTTON_START = 13
"""int: Button for setting the control by the application """

BUTTON_BACK  = 12
"""int: Button for relinquishing the by from the application """

class GamepadMotorControl:
    """
    A class for interfacing with a gamepad controller and mapping the controller
    button events to robot motion commands.

    Attributes:
        MAX_ANGULAR_VELOCITY (float): Maximum allowed angular velocity
        MAX_LINEAR_VELOCITY (float): Maximum allowed linear velocity
        config (str): File spwcifying the YAML configuration to setup the robot under control
        _stop_thread (bool): Flag to  control the control thread execution
        _control_on (bool): Flag to track if the gamepad controller has been enabled
        _robot (obj): A reference to the robot object context
        _rateHz (float): The rate at which the control thread samples the gamepad events
        thread (thread): COntrol thread handle

    """

    MAX_ANGULAR_VELOCITY = 2.0 # rad/s
    MAX_LINEAR_VELOCITY  = 0.4 # m/s

    def __init__(self, config=None, rateHz=25.0):
        """
        Initializes the internal state and acquires a reference to the robot object.
        Also resets the emergency flag within the robot object.

        Args:
            config (str): File spwcifying the YAML configuration to setup the robot under control
            rateHz (float): The rate at which the control thread samples the gamepad events

        Raises:
            ValueError: If 'config' is None.
            ValueError: If 'rateHz' is not a non-zero positive number

        """

        self.thread = None

        if config is None:
            raise ValueError("NULL configuratuin passed.")

        if rateHz <= 0:
            raise ValueError("Invalid rate specified.")

        try:
            self.gamepad = gp.Gamepad()
        except Exception as e:
            raise e

        self._rateHz = rateHz

        self._config      = config
        self._stop_thread = False
        self._control_on  = False
        self._robot       = get_robot_control_instance(self._config)
        self._robot.set_emergency_flag(True)

    def _control_thread(self):
        """
        Thread body implementing the morot command generation logic.

        This thread gets the gamepad at the rate determined by '_rateHz'. It then processes
        the events as per the state machine design and send out a command to the robot motor
        control interface under the following conditions:
        - The gamepad control event (BUTTON_Y) is set
        - The emergency stop condition event (Button_B) is reset

        """
        user_task_registered = False
        while self._stop_thread == False:
            # COLLECT GAMEPAD COMMANDS
            gp_data = self.gamepad.getData()

            # Mutiply joystick x axis by scuttle max angular velocity to get angular velocity
            angVel = gp_data[0] * GamepadMotorControl.MAX_ANGULAR_VELOCITY

            # Mutiply joystick y axis by scuttle max linear velocity to get linear velocity
            linVel  = gp_data[1] * GamepadMotorControl.MAX_LINEAR_VELOCITY
            
            # State machine logic
            if gp_data[BUTTON_Y] > 0:
                # Enable the gamepad controller and register with the robot control
                self._control_on = True
                self._robot.register_control_task(ID_CONTROLLER_TASK)
            elif gp_data[BUTTON_X] > 0:
                # Disable the gamepad controller 
                self._control_on = False
                self._robot.unregister_control_task(ID_CONTROLLER_TASK)
            elif gp_data[BUTTON_B] > 0:
                # Set the emergency stop condition
                self._robot.set_emergency_flag(True)
            elif gp_data[BUTTON_A] > 0:
                # Reset the emergency stop condition
                self._robot.set_emergency_flag(False)
            elif gp_data[BUTTON_START] > 0:
                # Enable the user task control and register with the robot control
                user_task_registered = True
                self._robot.register_control_task(ID_USER_TASK)
            elif gp_data[BUTTON_BACK] > 0:
                # Disable the user task control
                if user_task_registered:
                    user_task_registered = False
                    self._robot.unregister_control_task(ID_USER_TASK)

            # Send the command only if the gamepad control has been enabled
            if self._control_on:
                cmd = MotionCommand([linVel, angVel], tskId=ID_CONTROLLER_TASK)
                self._robot.sendCommand(cmd)

            # Honor the rate
            time.sleep(1.0/self._rateHz)

    def start(self):
        """
        Starts a processing thread if one is already not running.

        Returns:
            0 if successful, a negative value otherwise.

        """
        if self.thread == None:
            # Get the global instance of the robot control object
            if self._robot == None:
                self._robot = get_robot_control_instance(self._config)

            if self._robot == None:
                print("Failed to get robot control instance")
                return -1

            self.thread = threading.Thread(target=self._control_thread)
            self.thread.start()

            return 0

    def wait_for_exit(self):
        """
        A blocking call for the processing thread to exit. This method is called from
        the 'stop' method so must not be called in the same thread of execution as 'stop'.

        This method is provided so that the caller can block from exiting the program if
        another way of preventing the exit is not available. Since a thread is spawned inside
        this class, the user of the object must not exit until the thread is terminated and this
        method helps address that situation.

        This method will release the reference to the robot object.

        """

        self.thread.join()

        # Release the instance of the robot control object
        release_robot_control_instance()
        self._robot = None
        self.thread = None

    def stop(self):
        """
        Stops the processing thread, if one is running. The call will wait for the thread to exit
        before returning so will block until the thread is exited.

        The call will return immediately if no processing thread is being executed.

        """

        if self.thread != None:
            self._stop_thread = True
            self.wait_for_exit()

    def __del__(self):
        self.stop()

if __name__ == "__main__":
    config = '/opt/robot/edgeai-robotics-demos/python/common/scuttlepy/config/scuttle_sk_config.yaml'

    try:
        # Get the global instance of the robot control object
        motor_control = GamepadMotorControl(config=config)
        status = motor_control.start()
        if status < 0:
            sys.exit(status)

        motor_control.wait_for_exit()
    except KeyboardInterrupt:
        motor_control.stop()
        
