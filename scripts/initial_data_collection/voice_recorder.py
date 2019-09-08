#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import pyaudio
import wave
import os

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

class voice_recorder:
    def __init__(self):
        rospy.Subscriber('/stop', Bool, self.signal_cb)
        rospy.Subscriber("/is_baseline",  Bool, self.is_baseline_cb)
        self.stop = 1
        self.stop_old = 1
        self.p = pyaudio.PyAudio()
        print("pyaudio started")
        self.frames = []  
        self.should_stop_recording = 0
        self.num_collections = 0

    def is_baseline_cb(self, msg):
        self.is_baseline = msg.data

    def signal_cb(self, msg):
        self.stop = msg.data
        if ((self.stop - self.stop_old) == -1):
            self.stop_old = self.stop
	    print("start recording voice")
            self.start_recording()
	    print("stop recording voice")
        elif ((self.stop - self.stop_old) == 1):
            self.stop_old = self.stop
            self.stop_recording()  

    def start_recording(self):
        self.should_stop_recording = 0
        self.frames = []
        stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=RATE, input=True, frames_per_buffer=CHUNK, input_device_index = 2)
        while self.should_stop_recording == 0:
            data = stream.read(CHUNK)
            self.frames.append(data)

        stream.close()
		
        dir_audio = rospy.get_param('~dir_audio', '/home/intuitivecomputing/collab-data/audio')
        if (self.is_baseline):
            dir_audio = dir_audio + "_collection_baseline"
        else:
            dir_audio = dir_audio + "_collection_fpv"
        if not os.path.exists(dir_audio):
            os.makedirs(dir_audio)

        file_audio = dir_audio + "/recording" + str(self.num_collections) + ".wav"
        wf = wave.open(file_audio, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(self.frames))
        wf.close()
        self.num_collections += 1

    def stop_recording(self):
        self.should_stop_recording = 1

def main():
    rospy.init_node("voice_recorder")
    voice_recorder()
    rospy.spin()

if __name__ == "__main__":
    main()
