import asyncio
import websockets
from subprocess import call, check_output
from io import BytesIO
import numpy as np
from scipy.io.wavfile import write
from os import remove, environ

#Function that converts from a byte array to a numpy array
def bytes_to_array(b):
    np_bytes = BytesIO(b)
    return np.load(np_bytes, allow_pickle=True)

#Managers Client requests, sending back a string containing the transcription of the Wav file
async def deepspeech_stt(websocket, path):
    model_folder = environ['MODEL_FOLDER'] + "/"
    model_pbmm = environ["MODEL_PBMM_NAME"]
    model_scorer = environ["MODEL_SCORER_NAME"]
    audio_file = "output.wav"

    async for message in websocket:
        print("Received message from client")
        data_array = bytes_to_array(message)
        write("temp.wav", int(environ['SAMPLE_RATE']), data_array)
        print("temp.wav created")
        #Converting the temp wav to a monochannel audio file
        call(["ffmpeg", "-loglevel", "warning", "-i", "temp.wav", "-y", "-ar", "16000", "-ac", "1", "output.wav"])
        #Parsing stt stdout to a string
        parsed_output = check_output(['deepspeech', '--model', model_folder + model_pbmm, "--scorer", model_folder + model_scorer, "--audio", audio_file])
        await websocket.send(parsed_output)

        #Removing temp files
        remove("output.wav")
        remove("temp.wav")

#Starting the WebServer Service
async def main():
    ip_addr = "0.0.0.0"
    port = environ['PORT']
    print("Serving on {}:{}".format(ip_addr, port))
    async with websockets.serve(deepspeech_stt, ip_addr, port):
        await asyncio.Future()  # run forever

loop = asyncio.get_event_loop()
loop.run_until_complete(main())