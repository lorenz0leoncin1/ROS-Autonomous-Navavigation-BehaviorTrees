#!/bin/sh

#This script automatically calls all stt and tts components necessary to run the RASA Dialogue Manager (italian version)

#You need to train with 'rasa train' atleast once before running this bash script

#Move into rasa workspace folder
cd ~/catkin_ws/src/navigation/[it]robot_package

#Booting roscore
gnome-terminal --tab -- roscore

echo "Roscore started"

#Launching rasa servers
gnome-terminal --title="RASA actions" --tab -- rasa run actions
#gnome-terminal --title="RASA server" --tab -- rasa run
gnome-terminal --title="RASA Interactive" --tab -- rasa interactive
echo "Waiting 30 seconds for RASA Servers to boot"

#Wait for servers to launch
sleep 60


echo "Launching ROS Nodes..."

#Launch ROS Node for TTS, STT, Dialogue Management and robot bringup
gnome-terminal --tab -- roslaunch turtlebot3_gazebo small_house.launch &
gnome-terminal --tab -- roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/lorenzoleoncini/catkin_ws/src/navigation/map.yaml

echo "ROS Nodes launched"