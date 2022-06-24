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
from common.subject_follower import SubjectFollower

from apps.edgeai_subject_follower.follower_demo_class import FollowerDemo

class CallbackObj:
    def __init__(self, config):
        self.thresh       = config['prob_threshold']
        self.target_class = config['target_class']
        log_level         = config['log_level']
        min_radius        = config['min_radius']
        target_radius     = config['target_radius']
        center_threshold  = config['center_threshold']
        self.input_width  = config['input_width']
        self.input_height = config['input_height']
        output_topic      = config['output_topic']
        twist_topic       = config['twist_topic']
        self.bridge       = CvBridge()

        # Create a logger object
        self.logger = create_logger(name='edgeai_subject_follower',
                                    level=log_level)
        
        if ('max_lin_vel' in config):
            SubjectFollower.maxLinVel = config['max_lin_vel']
        if ('max_ang_vel' in config):
            SubjectFollower.maxAngVel = config['max_ang_vel']

        self.follower = SubjectFollower(input_width=self.input_width,
                                        input_height=self.input_height,
                                        min_radius=min_radius,
                                        target_radius=target_radius,
                                        center_threshold=center_threshold)

        queue_size = 5

        # Create the image publisher
        self.imgPub  = rospy.Publisher(output_topic, Image, queue_size=queue_size)

        # Create the Twist message publisher
        self.twistPub = rospy.Publisher(twist_topic, Twist, queue_size=queue_size)

    def __call__(self, img, box, class_name, reset=False):
        chassisTargets = [0, -0.5]

        if (reset == False) and (self.target_class in class_name):
            x      = (box[0]+box[2])/2
            y      = (box[1]+box[3])/2
            radius = (box[2]-box[0])/2
            chassisTargets = self.follower.getChassisTargets(x, y, radius)
            self.pubCb(img, radius, chassisTargets)
        else:
            self.logger.info("T:{}".format(chassisTargets))

    def pubCb(self, image, radius, chassisTargets):
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

    def stop(self):
        # Publish the Twist message
        twist = Twist()
        twist.linear.x  = 0
        twist.linear.y  = 0
        twist.linear.z  = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.twistPub.publish(twist)

params = ['input_width', 'input_height', 'output_topic', 'twist_topic', 'log_level',
          'robot_config', 'prob_threshold', 'min_radius', 'target_radius',
          'center_threshold', 'target_class', 'process_config']

optional_params = ['max_lin_vel', 'max_ang_vel']


if __name__ == "__main__":
    config = {}
    try:
        rospy.init_node('edgeai_subject_follower', anonymous=True, disable_signals=True)

        # Read configuration parameters
        for t in params:
            config[t] = rospy.get_param('~'+t)
        
        for t in optional_params:
            if rospy.has_param('~'+t):
                config[t] = rospy.get_param('~'+t)

        # Get the parameters for setting up the demo object
        with open(config['process_config'], 'r') as f:
            demo_config = yaml.safe_load(f)

        # Create a callback object and register it with the processing chain
        cbObj = CallbackObj(config)
        demo_config['callback_object'] = cbObj

        # Create the demoObj object
        demoObj = FollowerDemo(demo_config)

        # Start the demoObj
        demoObj.start()

        demoObj.wait_for_exit()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

    if cbObj:
        cbObj.stop()
        del cbObj

    if demoObj:
        demoObj.stop()
        del demoObj
