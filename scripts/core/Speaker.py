#!/usr/bin/env python
import pyttsx3
import rospy
from std_msgs.msg import String
import time


class Speaker(object):
    
    def __init__(self, speed=130, volume=1.0, voice_id=10):
        self.engine = pyttsx3.init()
        self.set_speed(speed)
        self.set_volume(volume)
        self.set_voice(voice_id)
        
        self.publisher = rospy.Publisher(
            "/home_edu/facial",
            String,
            queue_size=1,
            latch=True
        )
    
    def set_speed(self, speed):
        self.engine.setProperty("rate", speed)
    
    def set_volume(self, volume):
        self.engine.setProperty("volume", volume)
    
    def set_voice(self, voice_id):
        voices = self.engine.getProperty("voices")
        # print(len(voices))
        if voice_id >= 0 and voice_id < len(voices):
            voice = voices[voice_id]
            self.engine.setProperty("voice", voice.id)
        else:
            print("No voice id: ", voice_id)
    
    def say(self, msg, facial_start="happy-1", facial_end="happy"):
        print(msg)
        cmd = facial_start + ":" + msg
        self.publisher.publish(cmd)
        
        self.engine.say(msg)
        self.engine.runAndWait()
        
        self.publisher.publish(facial_end + ":")


if __name__ == "__main__":
    rospy.init_node("home_edu_speaker", anonymous=True)
    P = Speaker(140, 1.0, 16)

    voices = P.engine.getProperty("voices")
    for i, v in enumerate(voices):
        print(i, v.languages, v.name)
    
    P.say("Hi, nice to meet you.")
    time.sleep(1)
    P.say("I am angry now!", facial_start="angry", facial_end="angry-1")
    time.sleep(1)
    P.say("I am fine. thank you.", facial_start="happy-1", facial_end="happy")
    time.sleep(1)
    P.say("Good afternoon, what's your name?", facial_start="happy-2", facial_end="happy")
    time.sleep(1)
    P.say("Good afternoon, what's your name, I am raspberry pi.")
    time.sleep(1)
    # for i in range(10, 18, 1):
    # 	P.set_voice(10)
    # 	print(i)
    # P.say("I see 1 man in this picture and 2 women in this picture.")
    P.say("Good afternoon, what's your name? I am raspberry pi. I am your best mini computer that can do a lot of things.")
# 2, 9, 10, 11, 14, 15, 17, 22
