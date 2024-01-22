#!/usr/bin/env python

from speech_and_text.srv import Text
from terminal_colored_print import colored_print
from std_msgs.msg import String
from simplejson import load, loads
from unidecode import unidecode
import rospy
import requests
from os import path
from flask import Flask, request
from threading import Thread
import sys

#RASA Dialogue node implemented with Callback Channel for RASA Open Source

#RASA Server Webhook URL for Callback Channel
url = 'http://localhost:5005/webhooks/callback/webhook'

#Path to HOME
homedir = path.expanduser("~")
#Configuration, assuming default path to package
config_path = homedir + "/catkin_ws/src/speech_and_text/scripts/DialogueManagement/RASA-Dialogue/"

#Opening and reading configuration file, you can modify STT Topic and TTS Service used
config_file = open( config_path + 'config.json', 'r')
tts_service = load(config_file)['service']
config_file.seek(0)
stt_topic = load(config_file)['topic']

#Only true when human and bot are comunicating
is_interacting = False

#Keyword used when initializing a dialogue
keyword = 'computer'

#TTS Client request when bot utters a message
def tts_client(text_str):
    rospy.wait_for_service(tts_service)
    try:
        service = rospy.ServiceProxy(tts_service, Text)
        
        resp = service(text_str)
        return resp.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
#Callback wehn Vosk published audio transcription on stt_text topic
def callback(data):
    global is_interacting

    #Recognizing KEYWORD
    first_word = data.data.split(' ')[0]

    if (first_word == keyword or is_interacting):
        
        if (is_interacting): colored_print("Interaction is live", fg_color=209)

        if (first_word == keyword): 
            colored_print("Starting interaction!", fg_color=34)
            #Change i if u wanna use another language
            welcome_message = "computer"
            is_interacting = True
            user_message = unidecode(welcome_message.strip())
            colored_print("User uttered:", format="Bold")
            print(user_message)
            payload = '{"message": "'+user_message+'"}'
            # Set the timeout to 10 minutes (600 seconds)
            timeout_seconds = 6000
        else:
            user_message = unidecode(data.data.replace(keyword, "").strip())
            colored_print("User uttered:", format="Bold")
            print(user_message)
            payload = '{"message": "'+user_message+'"}'

            # Set the timeout to 10 minutes (600 seconds)
            timeout_seconds = 6000
        try:
            req = requests.post(url, data=payload, timeout=timeout_seconds)
            if (req.status_code != 200):
                colored_print("No answer from RASA Server, are you sure it is on?", fg_color=1)
        except requests.Timeout:
            colored_print("Request to RASA Server timed out after {} seconds".format(timeout_seconds), fg_color=1)
        return
        
#Subscriber to Speech to Text Topic
def stt_subscriber():
    print("Waiting for strings on Topic {}".format(stt_topic))
    rospy.Subscriber(stt_topic, String, callback)
    rospy.spin()

#Flask minimal server to listen to RASA Server asynchronous callbacks (like timers and reminders)
def callback_http_listener():
    app = Flask(__name__)
    #Hiding Flask production warning
    colored_print("Hiding Flask production warning", format="Bold")
    cli = sys.modules['flask.cli']
    cli.show_server_banner = lambda *x: None
    #Route for callback from RASA
    @app.route('/bot', methods=['POST'])
    def result():
        global is_interacting
        rasa_message = loads(request.data)['text']
        #Print the utter message from rasa
        colored_print("RASA uttered:", format="Bold")
        print(rasa_message)
        #Stopping interaction when interaction_end signal is received
        if ('interaction_end' in rasa_message):
            colored_print("Stopping interaction", fg_color=4)
            is_interacting = False
            return 'success'
        #Calling tts_client for TTS request
        tts_client(rasa_message)
        return 'success'
    ip = "localhost"
    port = 5034
    colored_print("Flask Server is listening to RASA callbacks on {}:{}".format(ip, port), format="Reversed")
    #Running Callback listener on PORT 5034
    app.run(host=ip, port=port, debug=True, use_reloader=False)
    
if __name__ == '__main__':
    #Initializing ROS Node
    rospy.init_node('rasa_dialogue', anonymous=True)
    colored_print("ROS Node initialised", format="Bold")
    flask_thread = Thread(target=callback_http_listener)
    flask_thread.daemon = True
    flask_thread.start()
    stt_subscriber()