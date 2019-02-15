#!/usr/bin/env python

import rospy
from ros_myo.msg import EmgArray
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
import os

class myo_recorder():
    def __init__(self):
        rospy.Subscriber('/stop', Bool, self.signal_cb)
        self.stop = 1
        self.stop_old = 0
        self.counter = 1
        self.myodata = np.zeros((8,1))
        rospy.Subscriber("/myo_raw/myo_emg", EmgArray, self.myo_cb)
        self.should_stop_recording = 0
        self.num_collections = 0

    def signal_cb(self, msg):
        self.stop = msg.data
        if ((self.stop - self.stop_old) == -1):
            self.num_collections += 1
            self.stop_old = self.stop
        elif ((self.stop - self.stop_old) == 1):
            self.stop_old = self.stop
            self.stop_recording()  

    def myo_cb(self, msg):
        if (self.stop == 0):
            self.counter += 1
            emgArr = msg.data
            currdata = np.asarray(emgArr).reshape((8,1))
            self.myodata = np.concatenate((self.myodata, currdata), axis = 1)
        if (self.should_stop_recording == 1):
            status = self.Plot()
            if (status == 1):
                rospy.logwarn("Plot failed")
            self.should_stop_recording = 0

    def Plot(self):
        if (self.counter != 1):
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.plot(np.arange(self.counter),self.myodata[0],color='b',label = "Emg[0]")
            ax.plot(np.arange(self.counter),self.myodata[1],color='r',label = "Emg[1]")
            ax.plot(np.arange(self.counter),self.myodata[2],color='g',label = "Emg[2]")
            ax.plot(np.arange(self.counter),self.myodata[3],color='c',label = "Emg[3]")
            ax.plot(np.arange(self.counter),self.myodata[4],color='m',label = "Emg[4]")
            ax.plot(np.arange(self.counter),self.myodata[5],color='y',label = "Emg[5]")
            ax.plot(np.arange(self.counter),self.myodata[6],color='k',label = "Emg[6]")
            ax.plot(np.arange(self.counter),self.myodata[7],color='w',label = "Emg[7]")
            plt.legend(loc='upper left')
            plt.draw()
            dir_myo = rospy.get_param('~dir_myo', '/home/gopika/collab-data/myo')
            dir_myo = dir_myo + "-collection-" + str(self.num_collections) 
            if not os.path.exists(dir_myo):
                os.makedirs(dir_myo)
            file_myo = dir_myo + "/plot" + str(self.num_collections) + ".png"
            fig.savefig(file_myo)
            return 0

    def stop_recording(self):
        self.should_stop_recording = 1

def main():
    rospy.init_node("myo_recorder")
    myo_recorder()
    rospy.spin()

if __name__ == "__main__":
    main()
