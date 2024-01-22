# Scripted Dialogue Manager

Testing Vosk STT and Larynx TTS together.

![Dialogue Manager](https://i.imgur.com/s3vBZ0o.png)

Before running ```rosrun speech_and_text scripted_dialogue_manager.py``` you need to launch:
- ```rosrun speech_and_text vosk_stt_publisher.py```  #You need the Vosk docker container started obviously
- ```rosrun speech_and_text larynx_tts_service_<variant>.py``` #If you use the docker variant you need to start the Larynx docker container first

## Optional:
If you want a faster and better TTS you can try the online implementation using WebSpeech API.

You can change the TTS service type between these two:
- **Larynx TTS (offline)**; when executing manager.py insert as argv ```local_tts```, both Conda and Docker implementations are supported
- **WebSpeech API (online-only)**; when executing manager.py insert as argv ```cloud_tts```. You need to start the WebServer first with ```node app.js``` in webservers/WebSpeechWebServer.

## Debug commands

The manager only responds to sentences starting with *ross*
- What time is it?
- What are you?
- Timer 30 seconds
- Have a nice day!

## Conversation example

**Speaker:** *Ross timer 30 seconds*

**Ross:** *I just set a timer for 30 seconds*
