from asyncio import get_event_loop, Future
import websockets
from subprocess import call, check_output
from io import BytesIO
from numpy import load, frombuffer, int16
from scipy.io.wavfile import write
from os import listdir, remove, environ
from stt import Model
from wave import open

#Function that converts from a byte array to a numpy array
def bytes_to_array(b):
    np_bytes = BytesIO(b)
    return load(np_bytes, allow_pickle=True)

#Managers Client requests, sending back a string containing the transcription of the Wav file
async def coqui_stt(websocket, path):
    model_folder = environ['MODEL_FOLDER'] + "/"
    scorer = ""
    model = ""
    files = listdir(model_folder)
    audio_file = "output.wav"
    for file in files:
        if (".tflite" in file): model = file
        if (".scorer" in file): scorer = file

    async for message in websocket:
        print("Received message from client")
        data_array = bytes_to_array(message)
        write("temp.wav", int(environ['SAMPLE_RATE']), data_array)
        print("temp.wav created")
        #Converting the temp wav to a monochannel audio file
        call(["ffmpeg", "-loglevel", "warning", "-i", "temp.wav", "-y", "-ar", "16000", "-ac", "1", "output.wav"])
        #Using STT module

        #Setting up model folder
        ds = Model(model_folder + model)
        #Setting up scorer folder
        ds.enableExternalScorer(model_folder + scorer)
        #Opening WAV file
        fin = open(audio_file, "rb")
        audio = frombuffer(fin.readframes(fin.getnframes()), int16)
        fin.close()
        sentence = ds.stt(audio)
        print(sentence)
        await websocket.send(sentence)
#Starting the WebServer Service
async def main():
    ip_addr = "0.0.0.0"
    port = environ['PORT']
    print("Serving on {}:{}".format(ip_addr, port))
    async with websockets.serve(coqui_stt, ip_addr, port):
        await Future()  # run forever

loop = get_event_loop()
loop.run_until_complete(main())