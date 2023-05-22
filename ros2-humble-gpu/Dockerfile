FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

ARG USERNAME="root"

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV USER=$USERNAME
ENV USERNAME=$USERNAME

RUN groupadd -g 1000 $USERNAME && \
    useradd -m -s /bin/bash -u 1000 -g 1000 -d /home/$USERNAME $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME

RUN apt update && apt upgrade -y && \
    apt install -y \
    eog \
    git \
    terminator \
    vim \
    wget

USER $USERNAME
RUN mkdir -p /home/$USERNAME/raspicat_ws/src && \
    cd /home/$USERNAME/raspicat_ws/src && \
    git clone -b ros2 https://github.com/rt-net/raspicat_ros.git && \
    git clone -b ros2 https://github.com/rt-net/raspicat_description.git && \
    git clone -b ros2 https://github.com/rt-net/raspicat_sim.git && \
    git clone -b ros2 https://github.com/rt-net/raspicat_slam_navigation.git && \
    ./raspicat_sim/raspicat_gazebo/scripts/download_gazebo_models.sh && \
    rosdep update && \
    rosdep install -r -y --from-paths --ignore-src ./

WORKDIR /home/$USERNAME/raspicat_ws/
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \ 
    echo "source /opt/ros/humble/setup.sh" >>/home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/raspicat_ws/install/setup.bash" >>/home/$USERNAME/.bashrc

WORKDIR /home/$USERNAME/raspicat_ws/src
CMD ["/bin/bash"]
