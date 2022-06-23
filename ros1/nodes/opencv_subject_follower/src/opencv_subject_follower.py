import sys
import yaml
import signal

import logging

# ROS related
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# Local modules
from common.command_line_parse import *
from common.logger import *
from apps.opencv_subject_follower.opencv_subject_follower import Follower

def signal_handler(sig, frame):
    global followerNode
    print("Ctrl-C")
    followerNode.stop()
    sys.exit(0)

class RosFollowerNode:
    def __init__(self, config, logger):
        # Create an image publisher
        self.config  = config
        output_topic = self.config['output_topic']
        twist_topic  = self.config['twist_topic']
        self.follower = None
        self.bridge  = CvBridge()
        self.logger  = logger

        queue_size = 5

        # Create the image publisher
        self.imgPub  = rospy.Publisher(output_topic, Image, queue_size=queue_size)

        # Create the Twist message publisher
        self.twistPub = rospy.Publisher(twist_topic, Twist, queue_size=queue_size)

    def start(self):
        self.follower = Follower(config, self.logger)
        self.follower.registerCallback(self.pubCb)
        self.follower.start()

    def stop(self):
        if self.follower:
            self.follower.stop()

    def pubCb(self, data):
        image, radius, chassisTargets = data
        self.logger.info("R:{0:.2f}, T:{1}".format(radius, chassisTargets))

        # Publish the input image
        self.imgPub.publish(self.bridge.cv2_to_imgmsg(image))

        # Publish the Twist message
        twist = Twist()
        twist.linear.x  = chassisTargets[0]
        twist.linear.y  = 0
        twist.linear.z  = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = chassisTargets[1]

        self.twistPub.publish(twist)

params = ['output_topic', 'twist_topic', 'log_level', 'title', 'robot_config', 'source',
          'framerate', 'input_width', 'input_height', 'min_radius', 'target_radius',
          'center_threshold', 'hsv_min', 'hsv_max', 'input_topic' , 'format', 'is_compressed']

if __name__ == "__main__":
    global followerNode

    # Register SIGINT handler
    signal.signal(signal.SIGINT, signal_handler)

    config = {}
    try:
        rospy.init_node('opencv_subject_follower', anonymous=True, disable_signals=True)

        # Read configuration parameters
        for t in params:
            config[t] = rospy.get_param('~'+t)

        # Create a logger object
        logger = create_logger(name='opencv_subject_follower', level=config['log_level'])

        followerNode = RosFollowerNode(config, logger)
        followerNode.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

    followerNode.stop()
