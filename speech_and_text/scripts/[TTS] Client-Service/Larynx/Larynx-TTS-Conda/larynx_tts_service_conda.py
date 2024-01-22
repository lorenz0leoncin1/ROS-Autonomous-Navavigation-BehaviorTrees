#!/usr/bin/env python

from __future__ import print_function

from speech_and_text.srv import Text, TextResponse
import rospy
from os import path, listdir
from os.path import exists
from hashlib import md5
from subprocess import call, Popen, PIPE

#Defines voice used by Larynx
voice = "ljspeech"
#Defines voice quality
quality = "high"
#Defines if caching is enabled or not
caching = True
# Stores cached files names
cache_array = []

# Path folder to Cache files
homedir = path.expanduser("~")
path_to_cache = homedir + \
    "/catkin_ws/src/speech_and_text/scripts/[TTS] Client-Service/Larynx/Larynx-TTS-Conda/CachedMP3s/"

# Loads cached files MD5 in ./CachedMP3s
def load_cached_mp3s():
    global cache_array
    global path_to_cache
    for file in listdir(path_to_cache):
        cache_array.append(file.replace(".mp3", ""))

# Checks if message is already cached
def is_cached(message):
    global cache_array
    messageMd5 = md5(str.encode(message)).hexdigest()
    print("Message MD5: ", messageMd5)
    if (messageMd5 in cache_array): return messageMd5, True
    else: return messageMd5, False

# Converting the WAV Larynx result in MP3
def convert_to_mp3(messageMd5, file_path):
    global cache_array
    global voice
    global quality

    #Do not cache again the MP3 file if this one is already being created
    if (exists(file_path + ".mp3")):
        print("File is being cached, there's no need to cache it again")
        return
    
    print("Caching {} ...".format(messageMd5))

    #Convert Larynx Wav output to MP3 with ffmpeg
    call(["ffmpeg", "-loglevel", "quiet", "-i", file_path + ".wav", file_path + ".mp3"])
    #Delete the old wav
    call(["rm", file_path + ".wav"])

    #Adding the new MP3 file to cache_array
    cache_array.append(messageMd5)
    print("Audio file cached and converted with FFMPEG successfully!\n--- ---")


# Handling client request
def handle_text(req):
    global path_to_cache
    global voice
    global quality
    global caching
    
    print("Received Text from client, wait for TTS...")
    message = req.input.strip()
    if (message == ""):
        print("An empty sentence can't be played!")
        return TextResponse(False)

    # Checking if the message is already cached
    messageMd5, cached = is_cached(message)
    print("Message: {}, is_cached: {}".format(messageMd5, cached))

    # Path for file (without extension)
    file_path = path_to_cache + messageMd5
    print("Starting Larynx TTS CLI utility...")

    # If the message is not cached
    if (not cached):
        #Declaring SOX process, only used when caching is True
        process_sox = None
        #If caching is requested the Larynx output gets redirected to SOX, saving it as WAV
        if (caching):
            #Assigning SOX process, saves Larynx output to wav
            process_sox = Popen(["sox", "-t", "raw", "-r", "22050", "-b", "16", 
                "-c", "1", "-e", "signed-integer", "-", "-t", "wav", file_path + ".wav"], stdin = PIPE, bufsize=4096)       
        #Declaring Larynx Process, redirecting stdout to PIPE
        process_larynx = Popen(["larynx", "-v", voice, "-q", quality, 
        "--raw-stream", "\"{}\"".format(message)], stdout=PIPE, bufsize=4096)
        #Declaring Aplay Process, directing stdin to PIPE, it plays the audio
        process_aplay = Popen(["aplay", "-r", "22050", "-c", "1", "-f", "S16_LE"], stdin=PIPE, bufsize=4096)
        

        # When larynx completed its processing exit the loop
        while (True):
            data = process_larynx.stdout.read(4096)
            if (len(data) == 0): 
                process_larynx.stdout.close()
                process_aplay.stdin.close()
                #If the file has to be cached, I close sox stdin too
                if (caching): process_sox.stdin.close()
                break
            #Sending aplay the data to play
            process_aplay.stdin.write(data)
            #Sending SOX the data to create a WAV file (only if caching is requested)
        
        #Executes FFMPEG and converts the output to MP3
        if (caching): convert_to_mp3(messageMd5, file_path)
        
        #Parsing Larynx_process exit code
        larynx_exit_code = process_larynx.wait()
        if (larynx_exit_code != 0): 
            print("Error while executing Larynx TTS utility!")
            return TextResponse(False)
        else: print("Larynx TTS utility ended successfully")   
        
        #Parsing Aplay_process exit code
        aplay_exit_code = process_aplay.wait()
        if (aplay_exit_code != 0): 
            print("Error while playing the audio file with aplay!")
            return TextResponse(False)
        else: print("Aplay played the audio stream without errors") 

    # If the message is cached
    else:
        #If the file is already cached, just play it
        exit_code = call(["mpg123", "-q", file_path + ".mp3"])
        if (exit_code != 0): 
            print("Error while playing the audio file with mpg123")
            return TextResponse(False)
        else:  print("MPG123 played the audio file without errors")
    
    return TextResponse(True)

# Starting the ROS Node Service
def larynx_tts_service():
    rospy.init_node('larynx_tts_service', anonymous=True)
    rospy.Service('local_tts', Text, handle_text)
    print("Ready to play sentences")
    rospy.spin()

if __name__ == '__main__':
    load_cached_mp3s()    #Loading Cached MP3s from Cache folder
    larynx_tts_service()  #Starting the ROS Service