FROM alphacep/kaldi-vosk-server:latest

ENV MODEL_NAME vosk-model-small-it
ENV MODEL_VERSION 0.4
ENV VOSK_SAMPLE_RATE 16000

RUN mkdir /opt/${MODEL_NAME} \
   && cd /opt/${MODEL_NAME} \
   && wget -q https://alphacephei.com/vosk/models/${MODEL_NAME}-${MODEL_VERSION}.zip \
   && unzip ${MODEL_NAME}-${MODEL_VERSION}.zip \
   && mv ${MODEL_NAME}-${MODEL_VERSION} model \
   && rm ${MODEL_NAME}-${MODEL_VERSION}.zip

EXPOSE 2700
WORKDIR /opt/vosk-server/websocket
CMD [ "python3", "./asr_server.py", "/opt/vosk-model-small-it/model" ]