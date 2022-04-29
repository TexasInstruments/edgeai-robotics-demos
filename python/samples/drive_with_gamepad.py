# A demonstration program: drive with gamepad control.

# Import External programs
import sys

# Import Internal Programs
from common.gamepad_motor_control import *

def main():
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
        
if __name__ == "__main__":
    main()
