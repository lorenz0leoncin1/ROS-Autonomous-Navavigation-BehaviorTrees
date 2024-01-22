# Robot Voice Command

## Requirements

- [Rasa](https://rasa.com/docs/rasa/installation/installing-rasa-open-source/)
- [Speech-and-text ros package](https://github.com/aislabunimi/tesi.triennale.milano)
- [Docker](https://docs.docker.com/get-docker/) 
- ROS ([Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu))
- [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) simulation ros package
- gnome

## Setup
Must run  at least once before running <i>launch_my_dialogue.sh</i>:

- Inside <i>[it]robot_package</i> to train the model
    ```
    rasa train
    ```


- Inside folder <i>Vosk</i> of <i>speech-and-text</i> package you must build the Vosk container with
    ```
    docker build -t ubuntu-vosk-stt .
    ``` 
    and run it with
    ```
    docker run -d -p 2700:2700 --name vosk ubuntu-vosk-stt
    ``` 

- Inside the folder <i>Larynx-Dockerized</i> of <i>speech-and-text</i> package you must build the larynx container with
    ```
    docker build -t ubuntu-larynx-tts .
    ```
    and run it with
    ```
    docker run -d -p 8768:8768 --name larynx ubuntu-larynx-tts
    ```

- Must get Rasa Duckling Extractor container with 
    ```
    docker pull rasa/duckling
    ```
    and run it with
    ```
    docker run -d -p 8000:8000 --name rasa rasa/duckling
    ```


The rasa workspace folder in <i>launch_my_dialogue.sh</i> must match the <i>[it]robot_package</i> location.

Must replace <i>larynx_tts_service_dockerized.py</i>, <i>vosk_stt_publisher.py</i>, <i>rasa_dialogue.py</i> of <i>speech-and-text</i> package with the ones in this folder.

## Giraff Additional Requirements
Delete the two roslaunch commands of <i>launch_my_dialogue.sh</i> and add ```gnome --tab -- roslaunch mission_pkg <your_main.launch>```.