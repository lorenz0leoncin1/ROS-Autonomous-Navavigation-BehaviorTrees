FROM framilano/ubuntu-deepspeech-stt:latest

ENV MODEL_FOLDER English_STT_0.9.3
ENV MODEL_PBMM_NAME deepspeech-0.9.3-models.pbmm
ENV MODEL_SCORER_NAME deepspeech-0.9.3-models.scorer
ENV MODEL_VERSION v0.9.3
ENV SAMPLE_RATE 16000
ENV PORT 8767

RUN cd /opt/deepspeech-stt \
    && mkdir ${MODEL_FOLDER} \
    && cd ${MODEL_FOLDER} \
    && curl -LO https://github.com/mozilla/DeepSpeech/releases/download/${MODEL_VERSION}/${MODEL_PBMM_NAME} \
    && curl -LO https://github.com/mozilla/DeepSpeech/releases/download/${MODEL_VERSION}/${MODEL_SCORER_NAME}

EXPOSE ${PORT}
WORKDIR /opt/deepspeech-stt/
CMD [ "python3", "deepspeech_stt_webserver.py" ]