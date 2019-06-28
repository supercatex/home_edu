#!/usr/bin/env python
import rospy
from core import Speech2Text as Speech2Text
from std_msgs.msg import String

class ManiSpeech2text:
    def __init__(self, unit_amount=2):
        self.agent_status = ""
        self.unit_id = ""
        self.unit_status = ""
        self.units = []
        self.units_topic = []
        self.unit_amount = unit_amount
        for i in range(self.unit_amount):
            self.units.append(Speech2Text(topic=("/home_edu/L_unit%d" % i), unitid=("u%d" % i)))
            self.units_topic.append(rospy.Subscriber("/home_edu/L_unit%d", String, self.unit_callback, queue_size=1))
        
        self.agent_sub = rospy.Subscriber("/home_edu/agent", String, self.agent_command, queue_size=1)
    
    def agent_command(self, data):
        self.agent_status = data.data
    
    def find_waiting_units(self):
        for i in range(self.unit_amount):   
            pass
    
    def unit_callback(self, data):
        cmd = str(data.data).split(":")
        self.unit_status = cmd[2]
        self.unit_id = cmd[1]

