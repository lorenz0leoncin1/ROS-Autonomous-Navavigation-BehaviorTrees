# Deploy RASA Dialogue Management

## Requirements
- Ubuntu 18.04 with Pyhton 2.7.17 and Python 3.6.9
- Docker
- ROS Melodic 1.14.12

## Instructions
1. Clone this package repo inside your catkin workspace for ROS (~/catkin_ws/src)
2. Go to ```~/catkin_ws``` and prepare the workspace with
```
catkin_make
```

Right now you can avoid executing roscore and all rosrun commands described below, the bash script ```rasa_dialogue_it.sh``` will take care of them.

## Larynx TTS
```local_tts``` service is available through larynx-tts container.

Build the larynx tts image using the Dockerfile (edit it if necessary) available in ```speech_and_text/scripts/[TTS] Client-Service/Larynx/Larynx-TTS-Docker``` with the command:

```
docker build -t ubuntu-larynx-tts .
```

And run it with:

```
docker run -d -p 8768:8768 ubuntu-larynx-tts
```

With roscore enabled, launch the ROS Node for Larynx with ```rosrun speech_and_text larynx_tts_service_dockerized.py```
Dependencies for this script are available in ```speech_and_text/scripts/[TTS] Client-Service/Larynx/Larynx-TTS-Docker/requirements.txt```

Now the container is online, serving on port 8768 and ```local_tts``` service is available for ROS clients.

## Vosk STT
```stt_text``` topic is available through vosk-stt container

Build the vosk stt image using the Dockerfile (edit if necessary) available in ```speech_and_text/scripts/[STT] Publisher-Subscriber/Vosk``` with the command:

```
docker build -t ubuntu-vosk-stt .
```

And run it with:

```
docker run -d -p 2700:2700 ubuntu-vosk-stt
```

With roscore enabled, launch ROS Node for Vosk with ```rosrun speech_and_text vosk_stt_publisher.py```
Dependencies for this script are available in ```speech_and_text/scripts/[STT] Publisher-Subscriber/Vosk/requirements.txt```

Now the container is online, serving on port 2700 and ```vosk_stt``` topic is now available for ROS subscribers.

## DucklingHTTPExtractor
Entity extraction for RASA requires an additional Docker container. Retrieve it using:

```
docker pull rasa/duckling
```

And run it with:

```
docker run -d -p 8000:8000 rasa/duckling
```

Duckling HTTP Extractor is now serving on port 8000.

## RASA
RASA run natively on Python 3.6.9 on two separate instances, the first one dedicated to dialogue management and the second one for custom actions support.

Dependencies for the italian demo package for RASA are available in ```speech_and_text/webservers/Rasa-ROS-WebServer/requirements.txt```

Train the model first, running ```rasa train``` in ```speech_and_text/webservers/Rasa-ROS-WebServer/[it]test_package```

With roscore enabled, launch the ROS Node for RASA Dialogue Management with ```rosrun speech_and_text rasa_dialogue.py```
Dependencies for this script are available in ```speech_and_text/scripts/DialogueManagement/RASA-Dialogue/requirements.txt```

This flask server requires both ```local_tts``` service and ```stt_text``` topic online in ROS. Callbacks from RASA are sent to ```localhost:5034```

## Launching all the components
With all the components installed we can try the italian demo for RASA running in ```speech_and_text/scripts``` the bash script ```launch_rasa_dialogue.sh``` with the command:

```
./launch_rasa_dialogue.sh it <vosk_container_name> <larynx_container_name> <duckling_container_name>
```

The Dialogue Manager will only answer to commands starting with the keyword ```computer```

Right now the available skills for the italian demo are:
- ```Computer cerca su internet <stuff>```(parses wikipedia quick snippets from google search response)
- ```Computer timer <amount of time>``` (set timer for a certain amount of time)
- ```Computer avvia test di valutazione mentale``` (you don't need to repeat 'computer' everytime you need to answer for this test)
- ```Computer stop``` (stops running text-to-speech playback)




