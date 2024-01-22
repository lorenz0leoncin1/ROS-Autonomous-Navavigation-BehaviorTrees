FROM framilano/ubuntu-larynx-tts:latest

#On which port larynx will work
ENV PORT 8768
#Select the voice model used by larynx
ENV VOICE lisa
#Audio synthethisation quality
ENV QUALITY high
#Boolean that allows you to use the caching system
ENV CACHING True
#Cache folder max size in MB, ignored if CACHING is False
ENV CACHE_SIZE 100

EXPOSE ${PORT}
WORKDIR /opt/larynx-tts
#Creating a sample.wav file that forces the download of the selected voice model
RUN larynx -v ${VOICE} "Sample" > sample.wav    
CMD [ "python3", "larynx_tts_webserver.py" ]