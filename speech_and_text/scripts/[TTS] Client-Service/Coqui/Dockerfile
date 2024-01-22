FROM framilano/ubuntu-coqui-tts:latest

ENV PORT 8765

EXPOSE ${PORT}
WORKDIR /opt/coqui-tts
CMD [ "python3", "coqui_tts_webserver.py" ]