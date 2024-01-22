#!/bin/sh

#This script automatically calls all stt and tts components necessary to run the RASA Dialogue Manager (italian version)

#You need to train with 'rasa train' atleast once before running this bash script

#Move into rasa workspace folder
cd ~/catkin_ws/src/navigation/[it]robot_package

#Booting roscore
gnome-terminal --tab -- roscore

echo "Roscore started"

#Launching rasa servers
gnome-terminal --title="RASA Actions" --tab -- rasa run actions
gnome-terminal --title="RASA Server" --tab -- rasa run #in teoria sarebbero 10 minuti ma sembra non vederli...
#gnome-terminal --title="RASA Server" --tab -- rasa run --endpoints endpoints.yml #in teoria sarebbero 10 minuti ma sembra non vederli...#
#gnome-terminal --title="RASA Interactive" --tab -- rasa interactive 
echo "Waiting 30 seconds for RASA Servers to boot"

#Wait for servers to launch
sleep 30

echo "Rasa servers launched"

# Launch Vosk STT, Larynx TTS and Duckling Extractor Docker Containers
# Must run once:
#   docker run -d -p 2700:2700 --name vosk ubuntu-vosk-stt
#   docker run -d -p 8768:8768 --name larynx ubuntu-larynx-tts
#   docker run -d -p 8000:8000 --name rasa rasa/duckling
sudo docker start vosk
sudo docker start larynx 
sudo docker start rasa

echo "Waiting 5 seconds for Docker Containers to boot"

#Wait for them to boot
sleep 5
echo "Docker containers for STT and TTS launched"

echo "Launching ROS Nodes..."

#Launch ROS Node for TTS, STT, Dialogue Management and robot bringup
gnome-terminal --tab -- rosrun speech_and_text larynx_tts_service_dockerized.py &
gnome-terminal --tab -- rosrun speech_and_text vosk_stt_publisher.py &
gnome-terminal --tab  -- rosrun speech_and_text rasa_dialogue.py &
gnome-terminal --tab -- roslaunch turtlebot3_gazebo small_house.launch &
gnome-terminal --tab -- roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/lorenzoleoncini/catkin_ws/src/navigation/map.yaml

echo "ROS Nodes launched"
