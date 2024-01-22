#!/usr/bin/env python

# Listening to Voice on topic and parsing text from it
# It requires Vosk WebServer active

import rospy
import websocket
from json import loads
import pyaudio
from std_msgs.msg import String
from speech_and_text.srv import Text, TextResponse

try:
    import thread
except ImportError:
    import _thread as thread

# Response queue
response = []
# WebSocket global variable
ws = None
# Keep listening indefinitely
listen = True
# Publisher channel
pub = rospy.Publisher('stt_text', String, queue_size=10)
# Pause conversation
pause = False

# Activity started every time the Vosk WebServer responds
def on_message(ws, message):
    partial = ""
    total = ""
    if ('partial' in message):
        partial = loads(message)['partial']
        if (partial == ""): return
        else: print("Partial: ", partial)
    if ("result" in message):
        total = loads(message)['text']
        print("Total:", total)
        response.append(total) #Only saving total results

# Manages errors during connection with the WebServer on the Docker Container
def on_error(ws, error):
    print(error)

# Activities executed when the WebSocket communication is closed
def on_close(ws):
    global listen
    listen = False
    print("### closed ###")

# Activities executed when the communication with the docker container has started
def on_open(ws):
    print("Comunication with Vosk WebServer started successfully")


def start_listening():
    global ws
    global listen

    # stick to this parameter or it will "fail" in a weird way (no error, but also no answers)
    RATE = 16000
    CHUNK = 8000
    BUFF_SIZE = 4000
    microphone = pyaudio.PyAudio()
    microphone_input = microphone.open(format=pyaudio.paInt16, channels=1, rate=RATE,
                input=True, frames_per_buffer=CHUNK)

    print("I'm listening now")

    def run():
        global pause
        while listen:
            if pause: microphone_input.stop_stream()
            if not pause:
                if microphone_input.is_stopped(): microphone_input.start_stream()
                data = microphone_input.read(BUFF_SIZE)
                try: ws.send(data, opcode=websocket.ABNF.OPCODE_BINARY)
                except:
                    print("You should probably start Vosk Docker Container first")
                    return
    thread.start_new_thread(run, ())
    rate = rospy.Rate(5)  # ROS Rate at 5Hz

    while not rospy.is_shutdown():
        if response:
            popstr = response.pop()
            print("Sending to Text Topic: ", popstr)
            pub.publish(popstr)
        rate.sleep()

def handle_microphone(req):
    global pause
    
    message = req.input
    if message == 'start':  
        pause = False
        return TextResponse(True)
    elif message == 'stop':
        pause = True
        return TextResponse(True)

    return TextResponse(False)


# Initializing the talker node
def vosk_stt_publisher():
    rospy.init_node('vosk_stt_publisher', anonymous=True)
    s = rospy.Service('handle_mic', Text, handle_microphone)
    start_listening()

# Separated thread managing the WebSocket connected to the Docker Vosk Server


def vosk_thread():
    global ws
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp("ws://0.0.0.0:2700/",
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()


# Listening to Vosk WebServer
if __name__ == '__main__':
    thread.start_new_thread(vosk_thread, ())
    
    vosk_stt_publisher()