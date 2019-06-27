import rospy
from std_msg.msg import String
from core import Speech2Text as speech2text

rospy.init_node("home_edu_agent", anonymous=True)

rate = rospy.Rate()



def main_callback(data):
    global _isListen
    _isListen = data.data


def L1_callback(data):
    global _isProcess_L1
    _isProcess_L1 = data.data


def L2_callback(data):
    global _isProcess_L2
    _isProcess_L2 = data.data


_isListen = ""
_isProcess_L1 = ""
_isProcess_L2 = ""

main_sub = rospy.Subscriber('/home_edu_listen/agent', String, main_callback, queue_size=1)
_L1_sub = rospy.Subscriber('/home_edu/L1', String, L1_callback, queue_size=1)
_L2_sub = rospy.Subscriber('/home_edu/L2', String, L2_callback, queue_size=1)

_main_pub = rospy.Publisuher("/home_edu_Listen/msg", String, queue_size=1)
_L1_pub = rospy.Publisher("/home_edu_listen/L1_situation", String, queue_size=1)
_L2_pub = rospy.Publisher("/home_edu_listen/L2_situation", String, queue_size=1)
