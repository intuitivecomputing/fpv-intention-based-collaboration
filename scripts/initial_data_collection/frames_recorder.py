#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os

class frames_recorder:
    def __init__(self):
        rospy.Subscriber('/stop', Bool, self.signal_cb)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        rospy.Subscriber("/is_baseline",  Bool, self.is_baseline_cb)
        self.bridge = CvBridge()
        self.num_rgb = 0
        self.num_collections = 0
        self.stop = 1
        self.stop_old = 0
        self.is_baseline = 0

    def is_baseline_cb(self, msg):
        self.is_baseline = msg.data

    def signal_cb(self, msg):
        self.stop = msg.data
        if ((self.stop - self.stop_old) == 1):
            self.num_collections += 1
        self.stop_old = self.stop

    def image_cb(self, data):
        if (self.stop == 0):
            try:
                image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print e
            dir_rgb = rospy.get_param('~dir_rgb', '/home/gopika/collab-data/collab-rgb-frames')
            
            if (self.is_baseline):
                dir_rgb = dir_rgb + "_collection_baseline"
            else:                
                dir_rgb = dir_rgb + "_collection_fpv"

            if not os.path.exists(dir_rgb):
                os.makedirs(dir_rgb)
            file_rgb = dir_rgb + "/frame" + (str(self.num_rgb).zfill(8)) + '.png'
            cv2.imwrite(file_rgb, image)
            self.num_rgb += 1

def main():
    rospy.init_node("frames_recorder")
    frames_recorder()
    rospy.spin()

if __name__ == "__main__":
    main()
