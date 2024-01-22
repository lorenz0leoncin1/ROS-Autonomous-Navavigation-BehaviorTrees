#!/usr/bin/env python

from __future__ import print_function

from speech_and_text.srv import Text, TextResponse
import rospy
from os import path, system


def handle_text(req):
    print("Received Text from client, wait for TTS...")
    system("gtts-cli \"{}\" | ffplay -nodisp -autoexit -loglevel quiet -".format(req.input))
    print("Waiting for another text string...")
    return TextResponse(True)

def gtts_tts_service():
    rospy.init_node('gtts_tts_service', anonymous=True)
    s = rospy.Service('local_tts', Text, handle_text)
    print("Ready to play sentences")
    rospy.spin()

if __name__ == '__main__':
    gtts_tts_service()