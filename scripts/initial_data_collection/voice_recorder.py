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
        self.stop = 1
        self.stop_old = 0
        self.p = pyaudio.PyAudio()
        self.frames = []  
        self.should_stop_recording = 0
        self.num_collections = 0

    def signal_cb(self, msg):
        self.stop = msg.data
        if ((self.stop - self.stop_old) == -1):
            self.stop_old = self.stop
            self.num_collections += 1
            self.start_recording()
        elif ((self.stop - self.stop_old) == 1):
            self.stop_old = self.stop
            self.stop_recording()  

    def start_recording(self):
        self.should_stop_recording = 0
        self.frames = []
        stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=RATE, input=True, frames_per_buffer=CHUNK)
        
        while self.should_stop_recording == 0:
            data = stream.read(CHUNK)
            self.frames.append(data)

        stream.close()
        dir_audio = rospy.get_param('~dir_audio', '/home/gopika/collab-data/audio')
        dir_audio = dir_audio + "-collection-" + str(self.num_collections) 
        if not os.path.exists(dir_audio):
            os.makedirs(dir_audio)

        file_audio = dir_audio + "/recording" + str(self.num_collections) + ".wav"
        wf = wave.open(file_audio, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(self.frames))
        wf.close()

    def stop_recording(self):
        self.should_stop_recording = 1

def main():
    rospy.init_node("voice_recorder")
    voice_recorder()
    rospy.spin()

if __name__ == "__main__":
    main()