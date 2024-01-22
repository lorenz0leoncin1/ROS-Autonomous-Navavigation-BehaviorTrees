import websockets
from asyncio import get_event_loop, Future
from subprocess import call
from io import BytesIO
from numpy import save
from scipy.io.wavfile import read
from os import environ, remove

# Function that converts from a numpy array to a bytes array
def array_to_bytes(x):
    np_bytes = BytesIO()
    save(np_bytes, x, allow_pickle=True)
    return np_bytes.getvalue()

# Managers Client requests, sending back a bytes array containing the WAV file
async def coqui_tts(websocket, path):
    async for message in websocket:
        # Wrapping tts module in subprocess.call
        call(['tts', '--text', message, "--out_path", "temp.wav"])
        rate, data = read("temp.wav")
        print("WAV rate is {}".format(rate))
        # Converting back from numpy array to bytes array
        data = array_to_bytes(data)
        await websocket.send(data)
        print("WAV file sent to client")


# Starting the WebServer Service
async def main():
    ip_addr = "0.0.0.0"
    port = environ['PORT']
    print("Serving on {}:{}".format(ip_addr, port))
    async with websockets.serve(coqui_tts, ip_addr, port, ping_timeout=60):
        await Future()  # run forever

loop = get_event_loop()
loop.run_until_complete(main())
