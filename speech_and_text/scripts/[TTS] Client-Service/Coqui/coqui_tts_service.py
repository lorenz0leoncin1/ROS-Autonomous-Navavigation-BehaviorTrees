#!/usr/bin/env python

from __future__ import print_function

from speech_and_text.srv import Text, TextResponse
import rospy
from os import path
import rospy
from numpy import load
from os import path
import websocket
from subprocess import call
from scipy.io.wavfile import write
from io import BytesIO
from time import sleep
import re
try:
    import thread
except ImportError:
    import _thread as thread

# Path folder to save temp files
homedir = path.expanduser("~")
path_to_temp = homedir + \
    "/catkin_ws/src/speech_and_text/scripts/Client-Service/Coqui/Temp/"

# WebSocket global variable
ws = None

tts_done = False

#Function that converts from a bytes array to a numpy array
def bytes_to_array(b):
    np_bytes = BytesIO(b)
    return load(np_bytes, allow_pickle=True)

#Manages Server response and plays the WAV file
def on_message(ws, message):
    global tts_done
    print("Received response from WebServer")
    #Converting numpy array to byte array
    data_array = bytes_to_array(message)
    #Writing output.wav file to be played by ffplay
    write(path_to_temp + "output.wav", 22050, data_array)
    print("Playing output.wav...")
    #Calling ffplay to play the output.wav result
    call(["ffplay", "-nodisp", "-loglevel", "quiet",
         "-autoexit", path_to_temp + "output.wav"])
    tts_done = True

#Manages errors during connection with the WebServer on the Docker Container
def on_error(ws, error):
    print(error)

#Activities executed when the WebSocket communication is closed
def on_close(ws):
    print("### closed ###")

#Activities executed when the communication with the docker container has started
def on_open(ws):
    print("Connection to CoquiTTS WebSocket through Docker Container succeded")

#Handling client request
def handle_text(req):
    global tts_done
    print("Received Text from client, wait for TTS...")
    sentences = re.split("[.!?]+", req.input)
    for sentence in sentences:
        if (sentence == "" or sentence == " "): continue
        ws.send(sentence)
    while not tts_done:
        sleep(1)
    tts_done = False
    return TextResponse(True)

#Initializing the talker node
def coqui_tts_service():
    rospy.init_node('coqui_tts_service', anonymous=True)
    s = rospy.Service('local_tts', Text, handle_text)
    print("Ready to play sentences")
    rospy.spin()

# Separated thread managing the WebSocket connected to the Docker Coqui Server
def coqui_thread():
    global ws
    websocket.enableTrace(False)
    ip_addr = "0.0.0.0"
    port = "8765"
    ws = websocket.WebSocketApp("ws://{}:{}/".format(ip_addr, port),
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()

if __name__ == '__main__':
    thread.start_new_thread(coqui_thread, ())
    coqui_tts_service()