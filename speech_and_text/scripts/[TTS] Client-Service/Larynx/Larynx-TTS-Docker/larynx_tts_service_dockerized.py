#!/usr/bin/env python

# Larynx TTS Service Node
# Connects with a WebServer and waits for a WAV file stream (or an MP3) and
# feeds it to aplay or mpg123

from __future__ import print_function
from speech_and_text.srv import Text, TextResponse
from subprocess import Popen, PIPE
from threading import Event
from terminal_colored_print import colored_print
from os import path
import argparse
import rospy
import websocket

try:
    import thread
except ImportError:
    import _thread as thread

#Parser for command line arguments
parser = argparse.ArgumentParser()
# WebSocket object
ws = None
# Audio process process
process = None
# is_raw global variable, necessary while playing mp3 files
is_raw = True
# Waits before executing any play process until the format from the webserver has been received ()
format_received_event = Event()
# Waits for queue to fill
queue_not_empty = Event()
#Check if docker container with the larynx webserver is online
webserver_connection = Event()
# Message queue
queue = []
#Path to ~
home_dir = path.expanduser('~')
# Defines training mode and args parsing, no need for playback
parser.add_argument('--training', action='store_true',
                    help='Enable training mode with no audio playback, useful to build cache')
args = parser.parse_args()
is_training = args.training

# Manages Server response and plays the WAV file
def on_message(ws, message):
    global is_raw
    global process
    global format_received_event

    # If the actual file format is not set, we're waiting the webserver to tell us the format
    # The playback process is only created if we're not in training mode
    if (not format_received_event.is_set()):
        #print("If the actual file format is not set, we're waiting the webserver to tell us the format")
        if (message == "raw"):
            #print("message == raw")
            if (not is_training): process = Popen(["aplay", "-r", "22050", "-c", "1",
                            "-f", "S16_LE"], stdin=PIPE, bufsize=4096)
            is_raw = True
            format_received_event.set()  # Setting the event when the format is received
            return
        if (message == "mp3"):
            #print("message == mp3")
            if (not is_training): process = Popen(["mpg123", "-q", "-"], stdin=PIPE, bufsize=4096)
            is_raw = False
            format_received_event.set()  # Setting the event when the format is received
            return
    else:  # If we received the format we can start writing on process stdin
        if (len(message) == 0):
            #print("No bytes left, closing stding and aplay process")
            if (not is_training): process.stdin.close()  # No bytes left, closing stding and aplay process
            return
        #print("Only write to playback process if we're not in training mode")
        #Only write to playback process if we're not in training mode
        if (not is_training): process.stdin.write(message)

# Manages errors during connection with the WebServer on the Docker Container
def on_error(ws, error):
    global process
    process.stdin.close()
    print(error)

# Activities executed when the WebSocket communication is closed
def on_close(ws):
    global process
    process.stdin.close()
    print("### closed ###")

# Activities executed when the communication with the docker container has started
def on_open(ws):
    global webserver_connection
    webserver_connection.set()  #Allowing the TTS Node to start
    print("Connection to Larynx TTS WebSocket through Docker Container succeded")

def handle_asynchronous_stop():
    colored_print("Stopping TTS manually", fg_color=202)
    try: process.terminate()
    except: pass

# Handling clients requests
def handle_text(req):
    global queue
    global queue_not_empty

    # Stripping the input message
    message = req.input.strip()
    if (message == ""):
        colored_print("An empty sentence can't be played!", fg_color=196, format="Bold")
        return TextResponse(False)
    
    #Stopping TTS asynchronously when "tts_stop" is received
    if ("tts_stop" == message):
        print("Stopping TTS asynchronously when tts_stop is received")
        handle_asynchronous_stop()
        return TextResponse(True)

    #Enqueing the new message
    queue.append(message)
    colored_print("Received Text from client, wait for TTS...", fg_color=34)
    colored_print("There are {} messages in queue".format(len(queue)), fg_color=34)

    #The queue is not empty, queue_manager can start playing audios
    queue_not_empty.set()
    return TextResponse(True)

# Function that manages the queue message recevied from ROS Client for local_tts
# When a new message arrives, we wait for Larynx Server to check if the element is already present in cache
def queue_manager():
    global queue_not_empty
    global queue
    global ws
    global process
    global is_raw
    global format_received_event
    
    while (True):
        #Waiting for queue to fill
        queue_not_empty.wait()
        #Till the queue is empty
        while (len(queue) != 0):
            
            enqueued_element = queue.pop(0)

            try:
                rospy.wait_for_service('handle_mic')
                mic_srv = rospy.ServiceProxy('handle_mic', Text)
                res = mic_srv("stop")

                if not res.response: 
                    colored_print('Connection with vosk failed', fg_color=200)
                else: 
                    colored_print('Communication paused', fg_color=62, format="Bold")
            except:
                colored_print('Connection with vosk failed', fg_color=200)

            #If Custom Command tts_timer is received, play the timer ringtone
            if (enqueued_element == "tts_timer"):
                process = Popen(["mpg123", "-q", home_dir + "/catkin_ws/src/speech_and_text/scripts/[TTS] Client-Service/Larynx/Larynx-TTS-Docker/CustomMP3s/timer.mp3"])
                colored_print("Playing the custom timer ringtone", fg_color=202)
                exit_code = process.wait()
                continue
            
            # Checking if WS is closed
            try:
                ws.send(enqueued_element)
            except:
                colored_print("Failed to connect with the WebServer, check if the WebServer is online",
                    fg_color=32)
                continue
            
            colored_print("Waiting audio format type from WebServer ...", fg_color=62)
            

            # Wait for "mp3" or "raw" message from webserver...
            #print("Wait for mp3 or raw message from webserver...")
            format_received_event.wait()

            #print("Checking if we're in training mode, if so, it ignores playback instruction")
            #Checking if we're in training mode, if so, it ignores playback instruction
            if (is_training): 
                print("Training [{}]".format(enqueued_element))
                continue

            if (is_raw): colored_print("RAW format received, playing using APLAY", 
                fg_color=62)
            else: colored_print("MP3 format received, playing using MPG123", 
                fg_color=62)

            # Waiting for aplay process to finish before responding to the ROS Client (no overlapping)
            #print("Waiting for aplay process to finish before responding to the ROS Client")
            exit_code = process.wait()

            # Resetting the format_received_event, now we're waiting for new texts
            #print("Resetting the format_received_event, now we're waiting for new texts")
            format_received_event.clear()

            # Checking the Audio Process exit code
            if (exit_code == 0):
                if (is_raw): colored_print("APLAY terminated successfully with code: {}\n--- ---".format(exit_code), 
                    fg_color=62)
                else: colored_print( "MPG123 terminated successfully with code: {}\n--- ---".format(exit_code), 
                    fg_color=62)
            else: colored_print("Error in audio playing process", 
                fg_color=196, format="Bold")

            try:
                rospy.wait_for_service('handle_mic')
                hanlde_mic = rospy.ServiceProxy('handle_mic', Text)
                res = hanlde_mic("start")

                if res.response: colored_print('Comunication resumed', fg_color=62, format="Bold")
            except:
                pass
        
        #Queue is empty, start waiting again
        queue_not_empty.clear()
        colored_print("Queue is empty, waiting for new texts on Service local_tts", 
            fg_color=27)

# Starting the ROS Node Service
def larynx_tts_service():
    global webserver_connection
    print("Waiting for Larynx TTS Docker WebServer response...")
    if (webserver_connection.wait(timeout=10)):
        rospy.init_node('larynx_tts_service', anonymous=True)
        rospy.Service('local_tts', Text, handle_text)
        print("Ready to play sentences")
        rospy.spin()
    else:
        print("Larynx TTS Docker WebServer unavailable, are you sure you started the docker container?")

# Separated thread managing the WebSocket connected to the Docker Larynx WebServer
def larynx_thread():
    global ws
    websocket.enableTrace(False)
    ip_addr = "0.0.0.0"
    port = "8768"
    ws = websocket.WebSocketApp("ws://{}:{}/".format(ip_addr, port),
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()


if __name__ == '__main__':
    if (is_training): colored_print("Training Mode is ON, there will be no audio playback", bg_color=20)
    thread.start_new_thread(larynx_thread, ())  # Starting the WebSocket thread
    thread.start_new_thread(queue_manager, ())  # Starting the queue manager for server messages
    larynx_tts_service()  # Starting the ROS Service