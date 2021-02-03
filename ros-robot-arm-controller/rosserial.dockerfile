FROM ros:melodic-robot

RUN apt-get -y update && apt-get -y upgrade && apt-get -y install ros-melodic-rosserial