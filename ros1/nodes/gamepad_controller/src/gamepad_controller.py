import sys
import signal
import threading
import logging
import time

# ROS related
import rospy
from geometry_msgs.msg import Twist

# Local modules
import common.gamepad as gp
from common.command_line_parse import *
from common.logger import *

class GamepadNode:
    MAX_ANGULAR_VELOCITY = 2
    MAX_LINEAR_VELOCITY  = 0.4

    def __init__(self, config, logger):
        # Create an image publisher
        self.thread = None
        twist_topic = config['twist_topic']
        rate  = config['rate']
        self.logger = logger

        try:
            self.gamepad = gp.Gamepad()
        except Exception as e:
            raise e

        queue_size = 5

        # Create the Twist message publisher
        self.twistPub = rospy.Publisher(twist_topic, Twist, queue_size=queue_size)

        self.rate = rospy.Rate(rate)

    def _control_thread(self):
        prevLinVel = -1
        prevAngVel = -1

        while not rospy.is_shutdown():
            # COLLECT GAMEPAD COMMANDS
            gp_data = self.gamepad.getData()

            # Mutiply joystick x axis by scuttle max angular velocity to get angular velocity
            angVel = gp_data[0] * GamepadNode.MAX_ANGULAR_VELOCITY

            # Mutiply joystick y axis by scuttle max linear velocity to get linear velocity
            linVel  = gp_data[1] * GamepadNode.MAX_LINEAR_VELOCITY
            
            if (prevLinVel != linVel) or (prevAngVel != angVel):
                # Publish the Twist message
                twist = Twist()
                twist.linear.x  = linVel
                twist.linear.y  = 0
                twist.linear.z  = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = angVel

                self.twistPub.publish(twist)
                prevLinVel = linVel
                prevAngVel = angVel

            #time.sleep(1.0/self._rate)
            self.rate.sleep()

    def start(self):
        if self.thread == None:
            self.thread = threading.Thread(target=self._control_thread)
            self.thread.start()
            return 0

    def wait_for_exit(self):
        self.thread.join()

        # Release the instance of the robot control object
        self.thread = None

    def stop(self):
        self.wait_for_exit()

    def __del__(self):
        self.stop()

params = ['twist_topic', 'log_level', 'rate']

if __name__ == "__main__":
    global gamepadNode

    config = {}
    try:
        rospy.init_node('gamepad_controller', anonymous=True, disable_signals=True)

        # Read configuration parameters
        for t in params:
            config[t] = rospy.get_param('~'+t)

        # Create a logger object
        logger = create_logger(name='gamepad_controller', level=config['log_level'])

        gamepadNode = GamepadNode(config, logger)
        gamepadNode.start()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
