#!/usr/bin/env python

#Node that coordinates comunication between ChatterBot Webserver on Docker with ROS nodes for STT and TTS

import websocket
import rospy
from speech_and_text.srv import Text
from std_msgs.msg import String
from simplejson import load
from sys import exit
from terminal_colored_print import colored_print
from os import path
try:
    import thread
except ImportError:
    import _thread as thread

# WebSocket object
ws = None
# Checking if websocket connection is closed
websocket_connection = None
#Path to HOME
homedir = path.expanduser("~")
#Configuration, assuming default path to package
config_path = homedir + "/catkin_ws/src/speech_and_text/scripts/DialogueManagement/ChatterBot-Dialogue/"
#Opening and reading configuration file, you can modify STT Topic and TTS Service used
config_file = open( config_path + 'config.json', 'r')
tts_service = load(config_file)['service']
config_file.seek(0)
stt_topic = load(config_file)['topic']
#Keyword for stt
keyword = 'computer'

#Whenever a new sentence is recognized send it to ChatterBot WebServer
def stt_callback(data):
    global ws
    global websocket_connection
    if (not websocket_connection): exit(0)

    first_word = data.data.split(' ')[0]

    #Checking if the sentence contains the key word ROSS
    if (first_word == 'computer'):
        user_message = data.data.replace(keyword, "").strip()
        colored_print("User uttered:", format="Bold")
        print(user_message)
        ws.send(user_message)

#Subscriber that listen on Topic Text, Vosk should publish on there   
def stt_subscriber():
    print("Waiting for strings on Topic Text, check if Vosk is publishing")
    rospy.Subscriber(stt_topic, String, stt_callback)
    rospy.spin()
    

#TTS Client called when ChatterBot answers
def tts_client(text_str):
    rospy.wait_for_service(tts_service)
    try:
        service = rospy.ServiceProxy(tts_service, Text)
        resp = service(text_str)
        return resp.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Manages Server response and plays the WAV file
def on_message(ws, message):
    colored_print("ChatterBot uttered:", format="Bold")
    print(message)
    tts_client(message)

# Manages errors during connection with the WebServer on the Docker Container
def on_error(ws, error):
    print(error)

# Activities executed when the WebSocket communication is closed
def on_close(ws):
    global websocket_connection
    websocket_connection = False
    print("ChatterBot WebServer closed")

# Activities executed when the communication with the docker container has started
def on_open(ws):
    global websocket_connection
    websocket_connection = True
    print("Connection to ChatterBot Server through Docker Container succeded")

def chatterbot_thread():
    global ws
    websocket.enableTrace(False)
    ip_addr = "0.0.0.0"
    port = "8769"
    ws = websocket.WebSocketApp("ws://{}:{}/".format(ip_addr, port),
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()

if __name__ == "__main__":
    rospy.init_node('chatterbot_dialogue', anonymous=True)
    print("ChatterBot Dialogue ROS node initialised")
    thread.start_new_thread(chatterbot_thread, ())  # Starting the WebSocket thread
    stt_subscriber() # Listening to Vosk publishing

