# Webservers

These are the actual scripts running on the various [Docker Containers](https://hub.docker.com/u/framilano) created for this project.
They are necessary because of Ubuntu 18.04 ROS1 limits with comunication between Python2 and Python3 Nodes.

# Dialogue Management

RASA actually can work directly with Python 3.6.9, so on Ubuntu 18.04. To run RASA Dialogue Management you need to have rasa installed
```
pip3 install rasa
```
To run the RASA Demo you need to have both a ```local_tts``` service running (like larynx_tts_dockerized) and an stt publisher on Topic ```stt_text``` (like vosk_stt_publisher)

Go into Rasa-ROS-WebServer and run
```
- rasa train
- rasa run actions
- rasa run
```

This will create two webservers running on ```localhost:5055``` and ```localhost:5005```, the first one manages [custom actions](https://rasa.com/docs/rasa/custom-actions/), the second one wait https requests to ```localhost:5005/webhooks/callback/webhook``` and sends back responses to ```localhost:5034/bot```

To comunicate with these two webserver a ROS Node written with Python2 and Flask framework is necessary. 
In ```speech_and_text/scripts/DialogueManagement/RASA-Dialogue/rasa_dialogue.py``` you can find a demo for it. It calls both STT and TTS nodes and calls them whenever the RASA WebServer gives it a proper response.

# RASA Dialogue Management Model

![schema rasa](https://i.imgur.com/nYXz0yt.png)
