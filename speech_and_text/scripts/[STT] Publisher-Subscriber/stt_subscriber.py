#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard :\n%s", data.data)
    
def stt_subscriber():
    rospy.init_node('stt_subscriber', anonymous=True)
    print("Waiting for strings on Topic Text")
    rospy.Subscriber("stt_text", String, callback)
    rospy.spin()

if __name__ == '__main__':
    stt_subscriber()