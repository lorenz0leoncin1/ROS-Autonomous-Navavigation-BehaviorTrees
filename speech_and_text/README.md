# ROS-Speech-And-Text
ROS Melodic package written with Python2 to manage Speech-To-Text and Text-To-Speech offline.

[Link to the thesis](https://docs.google.com/document/d/1IoreIdPDzEJ-_AD5ZjawTIu1yIo8v9ETN0D5v5EtzDY/edit?usp=sharing)

# Docker Containers usage explanation
Here we can find the run commands used for every docker container created in this project.

There are six different containers involved in this project (each one with a proper Dockerfile):
- Ubuntu Coqui TTS ```docker run -d -p 8765:8765 ubuntu-coqui-tts```
- Ubuntu Larynx TTS ```docker run -d -p 8768:8768 ubuntu-larynx-tts```
- Ubuntu Vosk STT ```docker run -d -p 2700:2700 ubuntu-vosk-stt```
- Ubuntu Coqui STT ```docker run -d -p 8766:8766 ubuntu-coqui-stt```
- Ubuntu DeepSpeech STT ```docker run -d -p 8767:8767 ubuntu-deepspeech-stt```
- Ubuntu ChatterBot ROS ```docker run -d -p 8769:8769 ubuntu-chatterbot-ros```

An additional container is used to extract [entities](https://rasa.com/docs/rasa/nlu-training-data/#entities) during dialogues
- DucklingHTTPExtractor ```docker run -d -p 8000:8000 rasa/duckling```

# Dialogue Management
There are two approaches in this repo:
- A stateless one powered by [ChatterBot](https://chatterbot.readthedocs.io/en/stable/)
- A stateful one powered by [RASA](https://rasa.com/)

Dialogue managament is assigned as usual to an external webserver, ChatterBot has its own Docker Container, meanwhile, RASA can be launched with Python3 directly in Ubuntu 18.04 with Python 3.6.9. More info in /webservers

**Details on how to deploy and test RASA dialogue with Larynx TTS and Vosk STT are available [here](https://github.com/aislabunimi/tesi.triennale.milano/tree/main/webservers/Rasa-ROS-WebServer)**

# Text-to-Speech (Client-Service Pattern)

![TTS](https://i.imgur.com/iQyRHTe.png)

**Nodes** involved:
- (Client) tts_client.py
- (Service) <variant_name>_tts_service.py

**Service** involved:
- Text (input: string, response: bool)

#### Instructions 
1. Run ```roscore```
2. Run the Client with ```rosrun speech_and_text tts_client.py```
3. Run the Service with ```rosrun speech_and_text <variant_name>_tts_service.py```

## ***1 - Espeak***	

#### Dependencies
- [espeak](http://espeak.sourceforge.net/) ubuntu module to tts offline. ```sudo apt install espeak```
- [ffmpeg](https://www.ffmpeg.org/) for audio playing

#### ✔️ Pros
- Multi-language support
- Extremely fast with no virtualizations or containers

#### ❌ Cons
- Bad quality overall

## ***2 - Coqui TTS***	

#### Dependencies
- [Coqui TTS](https://github.com/coqui-ai/TTS) for TTS (included with the docker container)
- [ffmpeg](https://www.ffmpeg.org/) for audio playing
- [docker](https://docs.docker.com/engine/install/ubuntu/)

#### ✔️ Pros
- Good speaking quality
- Possibility to train your own model

#### ❌ Cons
- Only English is officially supported *right now*
- Slow on long texts, Wav generation can't be avoided and there's no streaming option

#### Instructions 
A Docker container with a webserver serving Coqui TTS is required.

I couldn't find one, so I created it [framilano/ubuntu-coqui-tts](https://hub.docker.com/repository/docker/framilano/ubuntu-coqui-tts).

Coqui TTS doesn't currently allow you to use different pre-trained models, [feature is WIP](https://coqui.ai/models). Default model is English.

You can automatically grab it in scripts/Client-Service/Coqui/:

```docker build -t ubuntu-coqui-tts .```

And then run it with the instructions in the first paragraph of this README

## ***3 - Larynx TTS with Miniconda3***	

#### Dependencies
- [Larynx TTS](https://github.com/rhasspy/larynx) for TTS (included with the docker container)
- [Miniconda 3.8](https://docs.conda.io/en/latest/miniconda.html) for Environment virtualization

#### ✔️ Pros
- Multi-language support
- Supports streaming without generating a Wav File

#### ❌ Cons
- Python3 and requires Ananconda environment virtualization on Ubuntu <= 18.04

## ***4 - Larynx TTS with Docker***	

#### Dependencies
- [Larynx TTS](https://github.com/rhasspy/larynx) for TTS (included with the docker container)
- [docker](https://docs.docker.com/engine/install/ubuntu/)

#### ✔️ Pros
- Multi-language support
- Supports streaming without generating a Wav File

#### ❌ Cons
- Requires docker webserver to be always active

## ***5 - Google TTS***	

This one doesn't have an offline implementation, I added it just for comparison, it doesn't always work though, it probably needs an actual AUTH KEY from Google Speech API

#### Dependencies
- [ffmpeg](https://www.ffmpeg.org/) for audio playing
- [gTTS](https://pypi.org/project/gTTS/) for online TTS

#### ✔️ Pros
- Good speaking quality
- Fast on long texts

#### ❌ Cons
- Online-only
- Requires an external API Key

# Speech-to-Text (Publisher-Subscriber Pattern)

![STT](https://i.imgur.com/hFLqgtG.png)

**Nodes** involved:
- (Publisher) voice_publisher.py
- (Subscriber) <variant_name>_listener.py

**Topics** involved:
- Voice (float32[])

1. Run ```roscore```
2. Run the publisher with ```rosrun speech_and_text voice_stt_publisher.py```
3. Run the subscriber with ```rosrun speech_and_text <variant_name>_stt_subscriber.py```

## ***1 - Coqui STT***	

#### Dependencies
- [Coqui STT](https://github.com/coqui-ai/STT) for speech to text recognition
- [docker](https://docs.docker.com/engine/install/ubuntu/)

#### Instructions 

Docker container serving a WebServer that sends back the transcription of a Wav file (as a numpy array).

There's already a similar [one](https://github.com/coqui-ai/STT-examples/tree/r1.0/python_websocket_server), but I preferred to create my own docker container for better mantenaince. Coqui STT allows you to change the language you want to transcript from, read the [DockerHub page](https://hub.docker.com/repository/docker/framilano/ubuntu-coqui-stt) for more info.

You can automatically grab it in scripts/Publisher-Subscriber/voice_subscribers/Coqui/:

```docker build -t ubuntu-coqui-stt .```

And then run it with the instructions in the first paragraph of this README

The compressed size it's ~467MB, uncompressed is ~1.26B. The size changes based on the pre-trained models you choose to build.

#### ✔️ Pros
- Good transcription quality
- Multi-language support
- Possibility to train your own model

#### ❌ Cons
- Only accepts WAV files as input and there's no "continuous" listening mode

## ***2 - Vosk STT***	

#### Dependencies
- [Vosk API](https://alphacephei.com/vosk/) for speech to text recognition (included with the docker container)
- [docker](https://docs.docker.com/engine/install/ubuntu/)

#### Instructions 

Modification of [Vosk WebServer](https://alphacephei.com/vosk/server) through a Dockerfile, it allows you to select your preferred language from [Vosk Models](https://alphacephei.com/vosk/models)

It implements continous listening when starting the ```vosk_stt_publisher.py``` node, when sentences are recognized they're automatically sent on Text Topic to ```vosk_stt_subscriber.py```

You can automatically grab it running in scripts/Publisher-Subscriber/voice_subscribers/Vosk:

```docker build -t ubuntu-vosk-stt .```

And then run it with the instructions in the first paragraph of this README

The compressed size it's ~400MB, uncompressed is ~788MB. The size changes based off the pre-trained models you choose to build.

#### ✔️ Pros
- Good transcription quality
- Multi-language support
- Supports partial and total result, allowing for a fast transcription on simple questions

#### ❌ Cons
- Requires dockerization through with a WebServer

## ***3 - Mozilla DeepSpeech***	

#### Dependencies
- [DeepSpeech](https://github.com/mozilla/DeepSpeech) for speech to text recognition, (included with the docker container)
- [docker](https://docs.docker.com/engine/install/ubuntu/)

#### Instructions 

Docker container serving a WebServer that sends back the transcription of a Wav file (as a numpy array). The WebServer uses [DeepSpeech STT](https://github.com/mozilla/DeepSpeech)

I only found the English pre-trained model.

You can automatically grab it in scripts/Publisher-Subscriber/voice_subscribers/DeepSpeech:

```docker build -t ubuntu-deepspeech-stt .```

And then run it with the instructions in the first paragraph of this README

The compressed size it's ~395MB, uncompressed is ~2.21GB. The size changes based off the pre-trained models you choose to build.

#### ✔️ Pros
- Good transcription quality

#### ❌ Cons
- Only supports English and doesn't allow for "continuous" listening

# WebSpeech API  (Extra)

Another always-online implementation of STT and TTS is available in /webservers/WebSpeechWebServer. You can launch the NodeJs Server with ```node app.js```

This implementation is split in two sections:

- A **Service** ```localhost:3000/tts``` named **cloud_tts** that received a String from a Client it plays it through the default browser on the machine

- A **Publisher** ```localhost:3000/stt``` that received continous audio playing through the browser it publishes the transcription on topic Text

#### ✔️ Pros
- Perfect transcription and speaking quality
- Multi-language support
- Supports partial and total result, allowing for a fast transcription on simple questions

#### ❌ Cons
- Online-only implementation
- Features are browser-only, no CLI implementation
