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
        rospy.Subscriber('/start', Bool, self.signal_cb)
        self.start = 0
        self.p = pyaudio.PyAudio()
        self.frames = []  

    def signal_cb(self, msg):
        self.start = msg.data
        if (self.start == 1):
            self.start_recording()
        elif (self.start == 0):
            self.stop_recording()

    def start_recording(self):
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=RATE, input=True, frames_per_buffer=CHUNK)

        while (self.start == 1):
            data = self.stream.read(CHUNK)
            self.frames.append(data)
            if (self.start == 0):
                self.stop_recording()
                break

        self.stream.close()
        dir_audio = "/home/gopika/collab-data/audio"
        if not os.path.exists(dir_audio):
            os.makedirs(dir_audio)
        file_audio = dir_audio + "/recording.wav"
        wf = wave.open(file_audio, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(self.frames))
        wf.close()

    def stop_recording(self):
        self.start = 0


def main():
    rospy.init_node("voice_recorder")
    voice_recorder()
    rospy.spin()

if __name__ == "__main__":
    main()