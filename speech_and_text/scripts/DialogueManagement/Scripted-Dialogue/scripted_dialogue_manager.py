#!/usr/bin/env python

# This ROS node is split between two parts:
# - A client that asks larynx_tts to speak sending on Service Text
# - A subscriber that listens to vosk text publishing on topic Text

import rospy
from std_msgs.msg import String
import readline     #If you don't import it, it cut off part of stdin (?)
from speech_and_text.srv import Text
import time
import sys

# TTS Service type, can be local_tts or cloud_tts
# local_tts uses larynx tts Service, meanwhile cloud_tts uses WebSpeech API (online-only)
tts_service = sys.argv[1]

print("You selected {}".format(tts_service))

#TTS Service called whenever we need our robot to speak
def tts_client(text_str):
    global tts_service
    print("TTS Service \"Text\" called")
    rospy.wait_for_service(tts_service)
    try:
        service = rospy.ServiceProxy(tts_service, Text)
        resp = service(text_str)
        return resp.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#Managing STT results
def stt_results(data):
    #Removing left and right spaces
    text = data.data.lower().strip()
    #Splitting the result for multi-word commands
    text_split = text.split()
    #Checking for empty string
    if (text == ""): return 
    #Checking if the first word is "ross", ending the process if not so
    if (text_split[0] != "ross"): return
    #Removing ross from the commands
    text_split.pop(0)
    #Recreating text removing left spaces after splitting the message
    text = text.split("ross")[1].strip()
    print("I heard: {}".format(text))

    #Debug commands

    if (text == "what time is it"):
        tts_client(time.strftime("It's %H : %M", time.localtime()))
        return
    elif (text == "what are you"):
        tts_client("I'm a ROS Node")
        return
    elif ("repeat after me" in text):
        tts_client(text.split("repeat after me", 1)[1].strip())
        return
    elif ("timer" == text_split[0]):
        tts_client("I just set a timer for {}".format(text.split("timer", 1)[1]))
        return
    elif (text == "have a nice day"):
        tts_client("See you soon!")
        return
    
# Managing subscriber node on Text (requires Vosk publisher to be active)
def manager_subscriber():
    rospy.init_node('manager_subscriber', anonymous=True)
    print("Waiting for strings on Topic \"Text\"")
    rospy.Subscriber("text", String, stt_results)
    rospy.spin()

if __name__ == '__main__':
    manager_subscriber()