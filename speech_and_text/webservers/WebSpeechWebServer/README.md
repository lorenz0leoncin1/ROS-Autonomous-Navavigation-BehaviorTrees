## Implementation of WebSpeech API TTS / STT through a WebServer

![WebSpeech WebServer Model](https://i.imgur.com/95vmD5u.png)

This WebServer is a ROS node that offers two types of service:
* **STT Publisher-Subscriber** 

Listening your voice through ```localhost:3000/stt```, through socket.io the transcription reaches the webserver
and it get sent to the ROS Topic **Text**

* **TTS Client-Service** 

Whenever a *Text* request is received from a Client, the WebServer plays it through the browser on ```localhost:3000/tts?text=<text_req>```

The service name is **cloud_tts**

### Dependencies
* node

### Instructions

1. Run ```roscore```
3. Run the WebSpeech WebServer with ```node app.js```
