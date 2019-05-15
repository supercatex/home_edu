#!/usr/bin/env python
import rospy
from turtlebot_msgs.srv import SetFollowState


class ServiceController(object):
    
    def __init__(self, service_name="/turtlebot_follower/change_state", message_type=SetFollowState):
        self.service_name = service_name
        self.message_type = message_type
        self.items = dict()
    
    def register(self, key, service_name, message_type):
        rospy.wait_for_service(service_name)
        service_proxy = rospy.ServiceProxy(service_name, message_type)
        self.items[key] = {
            "service_proxy": service_proxy,
            "service_name": service_name,
            "message_type": message_type
        }
        
    def send_message(self, key, msg):
        rospy.wait_for_service(self.items[key]["service_name"])
        service_proxy = self.items[key]["service_proxy"]
        return service_proxy(msg)


if __name__ == "__main__":
    rospy.init_node("home_edu_follower", anonymous=True)
    rate = rospy.Rate(20)
    
    # 1. Create a ServiceController object.
    service = ServiceController()
    
    # 2. Register the service.
    service.register(
        "follower",
        "/turtlebot_follower/change_state",
        SetFollowState
    )
    
    # 3. Send message.
    service.send_message("follower", 1)
