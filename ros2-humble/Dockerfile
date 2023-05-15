FROM osrf/ros:humble-desktop-full

WORKDIR /root/

SHELL ["/bin/bash", "-c"]

RUN apt update && apt upgrade -y && \
  apt install -y \
  vim \
  git \
  terminator \
  wget

ARG DEFAULT_USER=default_user
ENV USERNAME=$DEFAULT_USER
RUN groupadd -g 1000 $DEFAULT_USER && \
    useradd -m -s /bin/bash -u 1000 -g 1000 -d /home/$DEFAULT_USER $DEFAULT_USER && \
    echo "$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R $DEFAULT_USER:$DEFAULT_USER /home/$DEFAULT_USER
  
RUN echo "source /opt/ros/humble/setup.sh" >> .bashrc

CMD ["/bin/bash"]
