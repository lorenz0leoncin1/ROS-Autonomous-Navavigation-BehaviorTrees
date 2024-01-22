import websockets
from hashlib import md5
from websockets.exceptions import ConnectionClosedError
from asyncio import get_event_loop, Future
from subprocess import Popen, PIPE, call, check_output
from os import environ, listdir, remove, utime
from os.path import exists, getmtime
from time import time, ctime


# WebServer that hosts Larynx TTS Engine, on a Text request it responds with the audio synthethization of the text.
# The audio is sent 4096 bytes at a time
# If it's not cached, the audio file is in RAW format (that can be an issue with some audio player)

# The following Environment Variables are required
# PORT (8768)
# VOICE (ljspeech)
# QUALITY (high)
# CACHING (True)
# CACHE_SIZE (100)

# By default cache is always used, based on global env CACHING it will continue saving new audios in the
# CachedMP3s folder
# If env CACHING is True new audios received will be saved
# If env CACHING is False only saved audios will be used, no new audio will be saved

#Path to CachedMP3s folder, all cached mp3s are store here
cache_folder = "./CachedMP3s/"
# Stores cached files names
cache_dict = {}
#Max cache folder size in MB
max_cache_size = float(environ['CACHE_SIZE'])
# Process used to synthethise audio from Text
process_larynx = None
# Process that saves the larynx stdout to a wav file
process_sox = None

# Loads cached files MD5 in ./CachedMP3s
def load_cached_mp3s():
    global cache_dict
    for file in listdir(cache_folder):
        cache_dict[file.replace(".mp3", "")] = getmtime(cache_folder + file)
    
    #removing the files with the oldest timestamp
    trim_cache()

    print("Current cache content:\n" + str(cache_dict))

# Checks if message is already cached
def is_cached(message):
    global cache_dict
    messageMd5 = md5(str.encode(message)).hexdigest()
    print("Message MD5: ", messageMd5)
    if (messageMd5 in cache_dict.keys()): return messageMd5, True
    else: return messageMd5, False

def get_file_with_oldest_usage():
    global cache_dict
    #sorted_dict is actually a list containing duples of cache_dict values
    sorted_dict = sorted(cache_dict.items(), key=lambda x: x[1])
    print("{} last modification is from {}".format(sorted_dict[0][0] + ".mp3", ctime(sorted_dict[0][1])))
    #return the first element of the list (oldest modification)
    return sorted_dict[0][0] + ".mp3"

#Check if cache_folder size is greater than max_cache_size
#If so, it removes elements with the oldest modification timestamps
#It stops when the cache_folder size is correct
def trim_cache():
    global cache_dict
    #There's no python utility that directly calculates the size of a folder
    #We use instead a CLI utility to find it, du
    #du -h (human readable) -k (uses kilobyte as reference) CachedMP3s/
    result = check_output(["du", "-h", "-k", cache_folder])
    current_size_in_kb = int(result.decode('utf-8').split()[0])

    #Till the folder size is greater than max_cache_size*1000
    while (current_size_in_kb > max_cache_size * 1000):
        file_name = get_file_with_oldest_usage()
        #Remove file from dictionary and delete it from file system
        remove("./CachedMP3s/" + file_name)
        cache_dict.pop(file_name.replace(".mp3", ""))
        print("Removing {} from cache".format(file_name))

        #Updating folder size
        result = check_output(["du", "-h", "-k", cache_folder])
        current_size_in_kb = int(result.decode('utf-8').split()[0])
        
# Converting the WAV Larynx result in MP3
def convert_to_mp3(messageMd5, file_path):
    global cache_dict
    global voice
    global quality

    # Do not cache again the MP3 file if this one is already being created
    if (exists(file_path + ".mp3")):
        print("File is being cached, there's no need to cache it again")
        return

    print("Caching {} ...".format(messageMd5))

    # Convert Larynx Wav output to MP3 with ffmpeg
    call(["ffmpeg", "-loglevel", "quiet", "-i",
         file_path + ".wav", file_path + ".mp3"])
    # Delete the old wav
    call(["rm", file_path + ".wav"])

    # Adding the new MP3 file to cache_dict and setting current timestamp as value
    cache_dict[messageMd5] = time()
    #If the cache size exceeded the max_cache_size value, then start
    #removing the files with the oldest timestamp
    trim_cache()
    print("Audio file cached and converted with FFMPEG successfully!\n--- ---")

# Sends audio and caches it in a WAV file contemporaneously, uses both larynx and sox
async def send_and_cache_audio(websocket, message, messageMd5, file_path):
    global process_sox
    global process_larynx
    # If caching is requested the Larynx output gets redirected to SOX, saving it as WAV
    process_sox = Popen(["sox", "-t", "raw", "-r", "22050", "-b", "16",
            "-c", "1", "-e", "signed-integer", "-", "-t", "wav", file_path + ".wav"], stdin=PIPE, bufsize=4096)
    # Piping audio file and sending fragments to client through websocket
    process_larynx = Popen(['larynx', '-v', environ["VOICE"], "-q", environ['QUALITY'],
            "--raw-stream", "\"{}\"".format(message)], stdout=PIPE, bufsize=4096)

    print("Sending and saving the audio file...")
    while True:
        data = process_larynx.stdout.read(4096)
        await websocket.send(data)
        process_sox.stdin.write(data)
        if (len(data) == 0):
            process_larynx.stdout.close()
            process_larynx.kill()
            process_sox.stdin.close()
            break
    print("Transmission completed")
    #Converts the WAV cached file to MP3
    convert_to_mp3(messageMd5, file_path)

# Sends the audio file in MP3 format from the CachedMP3s folder
async def send_cached_audio(websocket, file_path, messageMd5):
    global cache_dict
    #Cached file is being sent, updating its timestamp on cache_dict and on file_system
    current_timestamp = time()
    cache_dict[messageMd5] = current_timestamp
    #Changing both access and modification time with utime
    utime(file_path + ".mp3", (current_timestamp, current_timestamp))

    print


    mp3_file = open(file_path + ".mp3", "rb")
    print("Starting MP3 file transmission ...")
    while True:
        data = mp3_file.read(4096)
        await websocket.send(data)
        if (len(data) == 0):
            mp3_file.close()
            break
    print("Transmission completed\n--- ---")

# Sends audio without caching it, only Larynx is used
async def send_raw_audio(websocket, message):
    global process_larynx
    # Piping audio file and sending fragments to client through websocket
    process_larynx = Popen(['larynx', '-v', environ["VOICE"], "-q", environ['QUALITY'],
                            "--raw-stream", "\"{}\"".format(message)], stdout=PIPE, bufsize=4096)

    print("Starting WAV file transmission")
    while True:
        data = process_larynx.stdout.read(4096)
        await websocket.send(data)
        if (len(data) == 0):
            process_larynx.stdout.close()
            process_larynx.kill()
            break
    print("Transmission completed\n--- ---") 

# Sends through websocket the synthethised audio file from a message string
async def larynx_tts(websocket, path):
    global process_larynx
    global process_sox
    try:
        async for message in websocket:
            print("Received string:\n{}\n\n".format(message))

            # Checking if the message is already cached
            messageMd5, cached = is_cached(message)
            print("Message: {}, is_cached: {}".format(messageMd5, cached))

            # Path for the cached file
            file_path = cache_folder + messageMd5

            # If the message is not cached
            if (not cached):
                # Sending the client the actual format of the audio file, raw because it's not cached
                await websocket.send("raw")
                if (environ['CACHING'] == "True"):
                    # If caching is required send and cache the audio file
                    await send_and_cache_audio(websocket, message, messageMd5, file_path)
                else:
                    #If caching is not required just send the raw audio
                    await send_raw_audio(websocket, message)

            # If the message is cached
            else:
                # Sending the client the actual format of the audio file, raw because it's not cached
                await websocket.send("mp3")
                #Sends the cached audio
                await send_cached_audio(websocket, file_path, messageMd5)
    #When the client left suddenly
    except (ConnectionClosedError):
        print("Connection with Client interrupted")
        #If the clients interrupts the communication, kill all processes
        if(process_larynx != None):
            process_larynx.stdout.close()
            process_larynx.kill()
        if (environ['CACHING'] == 'True' and process_sox != None):
            process_sox.stdin.close()
            process_sox.kill()
            # Delete the incomplete wav file
        return  
    #Resetting the processes
    process_larynx = None
    process_sox = None


# Starting the WebServer Service
async def main():
    ip_addr = "0.0.0.0"
    port = environ['PORT']
    print("Serving on {}:{}".format(ip_addr, port))
    #Loading cached mp3s from CachedMP3s folder
    load_cached_mp3s()
    async with websockets.serve(larynx_tts, ip_addr, port, ping_timeout=None):
        await Future()  # run forever

loop = get_event_loop()
loop.run_until_complete(main())
