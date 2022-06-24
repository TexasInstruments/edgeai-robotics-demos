import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib
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
        self.thread = None
        self.img = None
        self.gotImg = False
    
    def make_subscriber(self, push_to_gst=False, gst_src=None):
        if (not push_to_gst):
            if (self.is_compressed == True):
                self.subscriber = rospy.Subscriber(self.topic, CompressedImage, self.compressed_callback, queue_size = 1)
            else:
                self.subscriber = rospy.Subscriber(self.topic, Image, self.raw_callback, queue_size = 1)
                self.bridge = CvBridge()
        else:
            if (self.is_compressed == True):
                self.subscriber = rospy.Subscriber(self.topic, CompressedImage, self.callback_with_push_to_gst, gst_src)
            else:
                self.subscriber = rospy.Subscriber(self.topic, Image, self.callback_with_push_to_gst, gst_src)
        

    def start(self):
        if self.thread == None:
            self.thread = threading.Thread(target=rospy.spin)
            self.thread.start()
        return

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
    
    def callback_with_push_to_gst(self,data,src):
        buf = Gst.Buffer.new_wrapped(data.data)
        src.emit("push-buffer",buf)
        self.gotImg = True
    
    def read(self):
        return self.gotImg, self.img