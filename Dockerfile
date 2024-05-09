# we build this docker based on a docker with ros already installed
FROM arm64v8/ros:noetic-perception

ARG PROXY=http://192.168.123.222:7890/


ENV http_proxy=$PROXY \
    https_proxy=$PROXY \
    HTTP_PROXY=$PROXY \
    HTTPS_PROXY=$PROXY
RUN echo "export http_proxy=$PROXY" >> /root/.bashrc
RUN echo "export https_proxy=$PROXY" >> /root/.bashrc
RUN echo "export HTTP_PROXY=$PROXY" >> /root/.bashrc
RUN echo "export HTTPS_PROXY=$PROXY" >> /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

RUN /bin/bash -c "source /root/.bashrc"


RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
    libglib2.0-dev \
    vim \
    libeigen3-dev \
    libgoogle-glog-dev \
    htop \
    tmux \
    git \
    python3-catkin-tools \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*
    

ENV UNITREE_PLATFORM=arm64
ENV ROBOT_TYPE=go2

ARG ROS_WS=/root/catkin_ws
RUN mkdir -p $ROS_WS/src


RUN apt-get update && apt-get install -y \
    libassimp-dev \
    ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-interactive-markers \
    ros-${ROS_DISTRO}-kdl-parser \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-controller \
    && rm -rf /var/lib/apt/lists/*

ARG SUPPORT_WS=/root/support_files
RUN mkdir -p $SUPPORT_WS 

WORKDIR $SUPPORT_WS
RUN git clone https://github.com/lcm-proj/lcm.git && \
    cd ${SUPPORT_WS}/lcm && \
    git checkout tags/v1.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12 && \
    make install 

WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/unitree_legged_sdk.git && \
    cd ${SUPPORT_WS}/unitree_legged_sdk && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12 

WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git && \
    cd ${SUPPORT_WS}/unitree_sdk2 && \
    ./install.sh && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make

WORKDIR $ROS_WS

