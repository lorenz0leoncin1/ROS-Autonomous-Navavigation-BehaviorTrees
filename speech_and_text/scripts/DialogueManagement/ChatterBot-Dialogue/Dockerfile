FROM framilano/ubuntu-chatterbot-ros:latest

ENV PORT 8769

EXPOSE ${PORT}
WORKDIR /opt/chatterbot-ros
CMD [ "python3", "chatterbot_ros_webserver.py" ]