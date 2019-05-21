#!/usr/bin/env python
import pyttsx3
import rospy
from std_msgs.msg import String
import time
from Speech2Text import Speech2Text


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
    
    def say(self, msg, f1="happy-2", f2="happy", keep_message=False):
        print(msg)
        cmd = f1 + ":" + msg
        self.publisher.publish(cmd)
        
        self.engine.say(msg)
        self.engine.runAndWait()
        
        cmd = f2 + ":"
        if keep_message:
            cmd = cmd + msg
        self.publisher.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("home_edu_speaker", anonymous=True)
    P = Speaker(140, 1.0, 16)
    
    s = Speech2Text()
    s.ambient_noise()
    P.say("Hello, I am ready.")
    while True:
        print("ready")
        t = s.listen()
        print(t)
        if t == "goodbye":
            break
        if t.find("introduce") >= 0:
            P.say("Hi, nice to meet you.")
            time.sleep(1)
            P.say("I am PCMS home service robot.", "happy-1")
            time.sleep(1)
            P.say("Service robots assist people by doing household chores.", "smart")
            time.sleep(1)
            P.say("And I am your home assistant.")
        if t.find("thank") >= 0:
            P.say("this is my pleasure", "happy-2")
    print("bye-bye")
    P.say("bye-bye")
    
    # while not rospy.is_shutdown():
    #     P.say("Hi, nice to meet you.")
    #     time.sleep(1)
    #     P.say("I am PCMS home service robot.", "happy-1")
    #     time.sleep(1)
    #     P.say("Service robots assist human beings, typically by performing a job that is dirty, dull, distant, dangerous or repetitive, including household chores.", "smart")
    #     time.sleep(1)
    #     P.say("And I am your home assistant.")
    #     time.sleep(10)
