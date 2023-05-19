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
  
RUN mkdir -p /home/$DEFAULT_USER/raspicat_ws/src

WORKDIR /home/$DEFAULT_USER/raspicat_ws/src

RUN git clone -b ros2 https://github.com/rt-net/raspicat_ros.git && \
    git clone -b ros2 https://github.com/rt-net/raspicat_description.git && \
    git clone -b ros2 https://github.com/rt-net/raspicat_sim.git && \
    git clone -b ros2 https://github.com/rt-net/raspicat_slam_navigation.git

WORKDIR /home/$DEFAULT_USER/raspicat_ws

RUN rosdep update 
RUN rosdep install -r -y --from-paths --ignore-src ./ 
SHELL ["/bin/bash", "-c", "-l"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    . install/setup.bash

WORKDIR /home/$DEFAULT_USER/raspicat_ws/src/raspicat_sim/raspicat_gazebo/scripts/

RUN ./download_gazebo_models.sh

RUN echo "source /opt/ros/humble/setup.sh" >>/home/$DEFAULT_USER/.bashrc

WORKDIR /home/$DEFAULT_USER/raspicat_ws

CMD ["/bin/bash"]
