#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ListeningUnit import ListeningUnit
import speech_recognition as sr
import time


class ListeningAgent(object):

    def __init__(self, num_of_units=3, topic="/home_edu_listening_agent", unit_topic="/home_edu/listening_unit"):
        self.num_of_units = num_of_units
        self.topic_message = topic + "/message"
        self.topic_status = topic + "/status"
        self.publish_message = rospy.Publisher(
            self.topic_message,
            String,
            queue_size=1
        )
        self.publish_status = rospy.Publisher(
            self.topic_status,
            String,
            queue_size=1
        )
        self.unit_message_topics = []
        self.unit_status_topics = []
        self.unit_cmd_topics = []
        self.unit_status = []
        self.unit_cmds = []
        for i in range(self.num_of_units):
            topic_message = unit_topic + "/message/%d" % i
            topic_status = unit_topic + "/status/%d" % i
            topic_cmd = unit_topic + "/cmd/%d" % i
            self.unit_message_topics.append(topic_message)
            self.unit_status_topics.append(topic_status)
            self.unit_cmd_topics.append(topic_cmd)
            rospy.Subscriber(
                topic_message,
                String,
                self.message_callback,
                queue_size=1
            )
            rospy.Subscriber(
                topic_status,
                String,
                self.status_callback,
                queue_size=1
            )
            cmd = rospy.Publisher(
                topic_cmd,
                String,
                queue_size=1
            )
            self.unit_cmds.append(cmd)
            self.unit_status.append(ListeningUnit.TYPE_IDLE)

    def is_listening(self):
        for status in self.unit_status:
            if status == ListeningUnit.TYPE_LISTENING:
                return True
        return False

    def get_idle_unit_index(self):
        for i, status in enumerate(self.unit_status):
            if status == ListeningUnit.TYPE_IDLE:
                return i
        return -1

    def message_callback(self, response):
        data = response.data.split(":")
        print("ListeningAgent->Message: %s" % data)
        self.publish_message.publish(data[1])

    def status_callback(self, response):
        data = response.data.split(":")
        self.publish_status.publish(data[1])
        self.unit_status[int(data[0])] = data[1]
        print(self.unit_status)


if __name__ == "__main__":
    rospy.init_node("home_edu_listening_agent", anonymous=True)
    _rate = rospy.Rate(20)

    _num_of_units = rospy.get_param("~num_of_units")
    _unit_topic = rospy.get_param("~unit_topic")
    _agent = ListeningAgent(num_of_units=_num_of_units, unit_topic=_unit_topic)
    while True:
        if not _agent.is_listening():
            _index = _agent.get_idle_unit_index()
            if _index >= 0:
                _agent.unit_cmds[_index].publish("1")
                time.sleep(1)
        _rate.sleep()
