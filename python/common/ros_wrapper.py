import rospy
from sensor_msgs.msg import CompressedImage,Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading

class Ros:
    
    def __init__(self,topic,format,is_compressed):
        self.topic = topic
        self.format = format
        self.is_compressed = is_compressed
        #rospy.init_node('opencv_subject_follower_ros_node', anonymous=True)
        if (self.is_compressed == True):
            self.subscriber = rospy.Subscriber(self.topic, CompressedImage, self.compressed_callback, queue_size = 1)
        else:
            self.subscriber = rospy.Subscriber(self.topic, Image, self.raw_callback, queue_size = 1)
            self.bridge = CvBridge()
        self.thread = None
        self.img = None
        self.gotImg = False

    def start(self):
        if self.thread == None:
            self.thread = threading.Thread(target=rospy.spin())
            self.thread.start()

    def wait_for_exit(self):
        self.thread.join()
        self.thread = None

    def stop(self):
        if self.thread != None:
            self.wait_for_exit()

    def isOpened(self):
        return True
    
    def release(self):
        self.stop()

    def raw_callback(self,data):
        #Explore using mutex
        self.img = self.bridge.imgmsg_to_cv2(data) 
        self.gotImg = True

    def compressed_callback(self,data):
        #Explore using mutex
        np_arr = np.fromstring(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
        self.gotImg = True
    
    def read(self):
        #Explore using mutex
        return self.gotImg, self.img