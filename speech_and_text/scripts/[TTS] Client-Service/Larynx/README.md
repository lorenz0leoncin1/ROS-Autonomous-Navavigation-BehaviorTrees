# Larynx TTS Explained
There are two versions of Larynx TTS Service, a miniconda one and a docker one.

The whole TTS mechanishm is split in:
- **TTS Client**, sends String messages to the Service Server and wait for Service Server to reply
- **TTS Service Server** that receives the message from the Client and plays the audio synthetisation of the text.

The TTS Client is represented by the tts_client.py node.

The TTS Service Server can be represent by the larynx_tts_service_conda.py or the larynx_tts_service_dockerized.py (not both contemporaneously obviously)

They both have the exact same features:
- Queue management for TTS messages
- Optional caching for messages

These two implementations give the user a choice based on his system, you can use a virtualized machine like Docker or just use your system Python3 implementation (with or without Conda)

# Larynx TTS Dockerized
It requires Docker to work, the larynx process and caching have been handed to the larynx_tts_webserver.py, the ROS node comunicates with it and plays the audio stream accordingly with its format (MP3 or WAV).
The Docker container *contains* the WebServer and stores the cached MP3s inside itself.

![HowLarynxTTSWorks](https://i.imgur.com/SkppNxN.png)

# Larynx TTS Conda
It requires Miniconda > 3.8 to work (or a system that has Python>3.8 by default, like Ubuntu 20.04), working on Ubuntu 16.04 we're required to use Conda.
This service implementation manages the Larynx process directly in the same node with the service server, that means that even the caching process is managed by the node.
No Webservers and websockets are involved, everything is managed internally by the ROS node.
