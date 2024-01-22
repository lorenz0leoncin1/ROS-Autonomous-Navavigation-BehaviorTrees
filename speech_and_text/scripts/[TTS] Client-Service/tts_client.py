#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import readline     #If you don't import it, it cut off part of stdin (?)
from speech_and_text.srv import Text

tts_type = "local_tts"

def tts_client(text_str):
    rospy.wait_for_service(tts_type)
    try:
        service = rospy.ServiceProxy(tts_type, Text)
        resp = service(text_str)
        return resp.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    while True:
        text_str = raw_input("Insert a string:\n")
        if (text_str == "stop"): break
        print("TTS requested, waiting for a response from Service Server...")
        if (tts_client(text_str)):
            print("TTS request received from the Service Server")
        else: print("TTS request failed, Service Server refused the message")
    print("Ending tts_client")