FROM osrf/ros:humble-desktop-full

WORKDIR /root/

SHELL ["/bin/bash", "-c"]

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update

RUN apt-get install -y vim
RUN apt-get install -y git
RUN apt-get install -y terminator
RUN apt-get install -y wget

RUN echo "source /opt/ros/humble/setup.sh" >> .bashrc

ARG DEFAULT_USER=default_user
ENV USERNAME=$DEFAULT_USER
RUN groupadd -g 1000 $DEFAULT_USER && \
    useradd -m -s /bin/bash -u 1000 -g 1000 -d /home/$DEFAULT_USER $DEFAULT_USER && \
    echo "$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER

CMD ["/bin/bash"]


