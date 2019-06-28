#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ListeningUnit import ListeningUnit


class ListeningAgent(object):

    def __init__(self, num_of_units=3, topic="/home_edu_listening_agent"):
        self.num_of_units = num_of_units
        self.units = []
        self.topic_message = topic + "/message"
        self.topic_status = topic + "/status"
        self.publish_message = rospy.Publisher(
            self.topic_message,
            String,
            queue_size=1
        )
        self.publish_message = rospy.Publisher(
            self.topic_status,
            String,
            queue_size=1
        )

        for i in range(self.num_of_units):
            unit = ListeningUnit(index=i)
            rospy.Subscriber(
                unit.topic_message,
                String,
                self.message_callback,
                queue_size=1
            )
            rospy.Subscriber(
                unit.topic_status,
                String,
                self.status_callback,
                queue_size=1
            )
            self.units.append(unit)

    def is_listening(self):
        for unit in self.units:
            if unit.status == ListeningUnit.TYPE_LISTENING:
                return True
        return False

    def get_idle_unit(self):
        for unit in self.units:
            if unit.status == ListeningUnit.TYPE_IDLE:
                return unit
        return None

    def message_callback(self, response):
        data = response.data.split(":")
        print("ListeningAgent->Message: %s" % data)
        self.publish_message.publish(data[1])

    def status_callback(self, response):
        data = response.data.split(":")
        print("ListeningAgent->Status: %s" % data)
        self.publish_message.publish(data[1])


if __name__ == "__main__":
    rospy.init_node("home_edu_home_edu_listening_agent", anonymous=True)
    _rate = rospy.Rate(20)

    _agent = ListeningAgent()
    while True:
        if not _agent.is_listening():
            _unit = _agent.get_idle_unit()
            if _unit is not None:
                _unit.listen(timeout=None)
        _rate.sleep()
