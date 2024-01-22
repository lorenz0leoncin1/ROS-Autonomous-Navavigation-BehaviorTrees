#!/bin/sh

#This script automatically calls all stt and tts components necessary to run the RASA Dialogue Manager (english version)
#Language and Docker container names must be passed as command line arguments

#Move into rasa workspace folder
cd ~/catkin_ws/src/speech_and_text/webservers/Rasa-ROS-WebServer/[$1]test_package

#Booting roscore
gnome-terminal --tab -- roscore

echo "Roscore started"

#Launching rasa servers
#You need to train with 'rasa train' atleast once before running this bash script
gnome-terminal --tab -- rasa run actions
gnome-terminal --tab -- rasa run
echo "Waiting 60 seconds for RASA Servers to boot"

#Wait for servers to launch
sleep 60

echo "Rasa servers launched"

# Launch Vosk STT, Larynx TTS and Duckling Extractor Docker Containers
docker start $2
docker start $3
docker start $4
echo "Waiting 10 seconds forDocker Containers to boot"

#Wait for them to boot
sleep 10
echo "Docker containers for STT and TTS launched"

echo "Launching ROS Nodes..."

#Launch ROS Node for TTS, STT and Dialogue Management
gnome-terminal --tab -- rosrun speech_and_text larynx_tts_service_dockerized.py &
gnome-terminal --tab -- rosrun speech_and_text vosk_stt_publisher.py &
gnome-terminal --tab -- rosrun speech_and_text rasa_dialogue.py

echo "ROS Nodes launched"


