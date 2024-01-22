FROM framilano/ubuntu-coqui-stt:latest

ENV MODEL_FOLDER Italian_STT_v0.0.1
ENV LANG italian
ENV AUTHOR jaco-assistant
ENV MODEL_VERSION v0.0.1
ENV MODEL_TFLITE_NAME model
ENV MODEL_SCORER_NAME kenlm_it
ENV SAMPLE_RATE 16000
ENV PORT 8766

RUN cd /opt/coqui-stt \
    && mkdir ${MODEL_FOLDER} \
    && cd ${MODEL_FOLDER} \
    && wget https://github.com/coqui-ai/STT-models/releases/download/${LANG}/${AUTHOR}/${MODEL_VERSION}/${MODEL_TFLITE_NAME}.tflite \
    && wget https://github.com/coqui-ai/STT-models/releases/download/${LANG}/${AUTHOR}/${MODEL_VERSION}/${MODEL_SCORER_NAME}.scorer

EXPOSE ${PORT}
WORKDIR /opt/coqui-stt
CMD [ "python3", "coqui_stt_webserver.py" ]