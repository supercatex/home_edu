#!/usr/bin/env python
import speech_recognition as sr
import rospy
from std_msgs.msg import String


class ListeningUnit(object):

    TYPE_LOADING = "Loading"
    TYPE_IDLE = "Idle"
    TYPE_LISTENING = "Listening"
    TYPE_PROCESSING = "Processing"
    TYPE_ERROR = "Error"

    def __init__(self, index=0, topic="/home_edu/listening_unit"):
        self.status = 0
        self.index = index
        self.topic_message = "%s/message/%d" % (topic, index)
        self.topic_status = "%s/status/%d" % (topic, index)

        self.publish_message = rospy.Publisher(self.topic_message, String, queue_size=1)
        self.publish_status = rospy.Publisher(self.topic_status, String, queue_size=1)
        self.sr = sr.Recognizer()
        self.set_status(ListeningUnit.TYPE_IDLE)

    def format_content(self, content):
        return str(self.index) + ":" + content

    def set_status(self, listening_unit_type, message=""):
        print("ListeningUnit[%d]->%s: %s" % (self.index, listening_unit_type, message))
        self.status = listening_unit_type
        self.publish_status.publish(self.format_content(listening_unit_type))

    def send_message(self, message):
        print("ListeningUnit[%d]->Message:%s" % (self.index, message))
        self.publish_message.publish(self.format_content(message))
        self.set_status(ListeningUnit.TYPE_IDLE)

    def listen(self, timeout=None):
        try:
            with sr.Microphone() as source:
                self.set_status(ListeningUnit.TYPE_LOADING)
                self.sr.adjust_for_ambient_noise(source, duration=1)

                self.set_status(ListeningUnit.TYPE_LISTENING)
                audio = self.sr.listen(source, timeout=timeout)

                self.set_status(ListeningUnit.TYPE_PROCESSING)
                message = self.sr.recognize_google(audio, language="en-US")

                self.send_message(message)
                return message
        except Exception as e:
            self.set_status(ListeningUnit.TYPE_ERROR, e)
            self.set_status(ListeningUnit.TYPE_IDLE)
            return self.format_content("")


if __name__ == "__main__":
    rospy.init_node("home_edu_listening_unit", anonymous=True)

    u = ListeningUnit()
    u.listen(3)
