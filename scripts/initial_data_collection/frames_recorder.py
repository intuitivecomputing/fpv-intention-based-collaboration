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
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_cb)
        self.bridge = CvBridge()
        self.num_rgb = 0
        self.num_depth = 0
        self.num_collections = 0
        self.stop = 1
        self.stop_old = 0

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
            dir_rgb = dir_rgb + "-collection-" + str(self.num_collections) 
            if not os.path.exists(dir_rgb):
                os.makedirs(dir_rgb)
            file_rgb = dir_rgb + "/frame" + (str(self.num_rgb).zfill(8)) + '.png'
            cv2.imwrite(file_rgb, image)
            self.num_rgb += 1
    
    def depth_cb(self, data):
        if (self.stop == 0):
            try:
                # image = self.bridge.imgmsg_to_cv2(data, "16UC1")
                # depth_array = np.array(image, dtype=np.float32)
                # cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
                image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            except CvBridgeError as e:
                print e

            ## save image
            dir_depth = rospy.get_param('~dir_depth' ,'/home/gopika/collab-data/collab-depth-frames')
            dir_depth = dir_depth + "-collection-" + str(self.num_collections) 
            if not os.path.exists(dir_depth):
                os.makedirs(dir_depth)
            file_depth = dir_depth + "/frame" + (str(self.num_depth).zfill(8)) + '.png'
            # cv2.imwrite(file_depth, depth_array*255)
            cv2.imwrite(file_depth, image)
            self.num_depth += 1

def main():
    rospy.init_node("frames_recorder")
    frames_recorder()
    rospy.spin()

if __name__ == "__main__":
    main()
