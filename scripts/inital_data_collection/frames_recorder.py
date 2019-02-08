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
        rospy.Subscriber('/start', Bool, self.signal_cb)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_cb)
        self.bridge = CvBridge()
        self.num_rgb = 0
        self.num_depth = 0
        self.start = 0

    def signal_cb(self, msg):
        self.start = msg.data

    def image_cb(self, data):
        if (self.start == 1):
            try:
                image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print e
            dir_rgb = rospy.get_param('~dir_rgb', '/home/gopika/collab-data/collab-rgb-frames')
            if not os.path.exists(dir_rgb):
                os.makedirs(dir_rgb)
            file_rgb = dir_rgb + "/frame" + str(self.num_rgb) + '.png'
            cv2.imwrite(file_rgb, image)
            self.num_rgb += 1
    
    def depth_cb(self, data):
        if (self.start == 1):
            try:
                image = self.bridge.imgmsg_to_cv2(data, "16UC1")
                depth_array = np.array(image, dtype=np.float32)
                cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            except CvBridgeError as e:
                print e
            dir_depth = rospy.get_param('~dir_depth' ,'/home/gopika/collab-data/collab-depth-frames')
            if not os.path.exists(dir_depth):
                os.makedirs(dir_depth)
            file_depth = dir_depth + "/frame" + str(self.num_depth) + '.png'
            cv2.imwrite(file_depth, depth_array*255)
            self.num_depth += 1

def main():
    rospy.init_node("frames_recorder")
    frames_recorder()
    rospy.spin()

if __name__ == "__main__":
    main()
