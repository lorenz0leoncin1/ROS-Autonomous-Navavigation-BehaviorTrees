#!/usr/bin/env python

# Listening to numpy arrays on 'voice' and then transcripting them

import rospy
from numpy import save
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import websocket
from io import BytesIO
import sounddevice as sd
from std_msgs.msg import String
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
pub = rospy.Publisher('text', String, queue_size=10)
#Sample rate
fs = 16000

#Function that converts from a numpy array to a byte array
def array_to_bytes(x):
    np_bytes = BytesIO()
    save(np_bytes, x, allow_pickle=True)
    return np_bytes.getvalue()

#Manages Server response and plays the WAV file
def on_message(ws, message):
    print("Received message from Coqui webserver:")
    print(message.replace("\n", ""))
    response.append(message.replace("\n", ""))


#Manages errors during connection with the WebServer on the Docker Container
def on_error(ws, error):
    global listen
    listen = False
    print(error)

#Activities executed when the WebSocket communication is closed
def on_close(ws):
    global listen
    listen = False
    print("### closed ###")

#Activities executed when the communication with the docker container has started
def on_open(ws):
    print("Connection to CoquiSTT WebSocket succeded")

#Activities called when the voice_publisher publishes a numpy array
def start_listening():
    global ws
    global listen
    duration = 5 #seconds
    def run():
        print("Press a key to record for {} seconds:\n".format(duration))
        while listen:
            raw_input("\n")
            recording = sd.rec(int(duration * fs), samplerate=fs, channels=1)
            sd.wait()
            bytearray = array_to_bytes(recording)
            print("Sent recording to WebServer")
            ws.send(bytearray, opcode=websocket.ABNF.OPCODE_BINARY)
    thread.start_new_thread(run, ())

    rate = rospy.Rate(5)  # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        if response:
            popstr = response.pop()
            print("Sending to Text Topic: ", popstr)
            pub.publish(popstr)
        rate.sleep()

#Initializing the listener node
def coqui_stt_publisher():
    rospy.init_node('coqui_stt_publisher', anonymous=True)
    start_listening()

# Separated thread managing the WebSocket connected to the Docker Coqui Server
def coqui_thread():
    global ws
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp("ws://0.0.0.0:8766/",
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()


if __name__ == '__main__':
    thread.start_new_thread(coqui_thread, ())
    coqui_stt_publisher()
