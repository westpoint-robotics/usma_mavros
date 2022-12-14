FROM px4io/px4-dev-ros-melodic:2021-08-18
LABEL Description="ROS-Melodic-PX4"
LABEL maintainer Steve Henderson

ENV DEBIAN_FRONTEND noninteractive 
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
ros-melodic-octomap-* \
ros-melodic-image-view \
ros-melodic-stereo-image-proc \
ros-melodic-rqt \
ros-melodic-rqt-common-plugins \
gedit \
psmisc \
&& apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-mavros-msgs vim wget screen

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh

# Configure ROS
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

WORKDIR /
COPY mavros_px4_entry.sh /
RUN chmod +x /mavros_px4_entry.sh

ENV FCUURL="udp://:14540@127.0.0.1:14557"
ARG FCUURL=$FCUURL

ENV ROS_MASTER_URI="http://localhost:11311"
ARG ROS_MASTER_URI=$ROS_MASTER_URI

ENV STARTUP_DELAY=5
ARG STARTUP_DELAY=$STARTUP_DELAY

ENTRYPOINT /mavros_px4_entry.sh ${FCUURL} ${STARTUP_DELAY}