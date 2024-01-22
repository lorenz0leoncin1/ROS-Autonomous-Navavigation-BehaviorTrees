#!/usr/bin/env python

from __future__ import print_function
from subprocess import call
from speech_and_text.srv import Text, TextResponse
import rospy


def handle_text(req):
    print("Received Text from client, wait for TTS...")
    call(["espeak", req.input])   
    print("Waiting for another text string...")
    return TextResponse(True)

def espeak_tts_service():
    rospy.init_node('espeak_tts_service', anonymous=True)
    s = rospy.Service('local_tts', Text, handle_text)
    print("Ready to play sentences")
    rospy.spin()

if __name__ == '__main__':
    espeak_tts_service()