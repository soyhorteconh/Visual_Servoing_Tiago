FROM ubuntu:18.04

#System full upgrade
RUN apt-get update && apt-get --with-new-pkgs upgrade -y

#Essential packages
RUN apt-get update && apt-get install -y apt-utils
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris
RUN apt-get update && apt-get install -y --fix-missing \
    git vim curl build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev \
    libopenblas-dev libatlas-base-dev cmake make lsb-release  \
    sudo ca-certificates gnupg-agent libssl-dev apt-transport-https \
    software-properties-common usbutils

#Install ROS-melodic
RUN \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update
RUN apt-get update && apt-get -y -o Dpkg::Options::="--force-overwrite" dist-upgrade
RUN apt-get update && apt-get install -y \
    ros-melodic-catkin python3-catkin-tools ros-melodic-geometry-msgs ros-melodic-common-msgs ros-melodic-roscpp \
    ros-melodic-image-transport ros-melodic-image-geometry ros-melodic-nodelet ros-melodic-tf2 ros-melodic-tf2-ros \
    ros-melodic-tf2-geometry-msgs ros-melodic-cv-bridge ros-melodic-camera-info-manager ros-melodic-angles ros-melodic-xacro \
    ros-melodic-joint-state-publisher ros-melodic-robot-state-publisher ros-melodic-rgbd-launch
RUN apt-get install -y \
    ros-melodic-cmake-modules
RUN apt-get install -y \
    ros-melodic-pcl-conversions
RUN apt install -y \
    ros-melodic-rosbash

#Install rqt
RUN apt-get install -y \
    ros-melodic-rqt ros-melodic-rqt-common-plugins
#Install rviz
RUN apt-get install -y \
    ros-melodic-rviz
#Install multiplot
RUN apt-get update && apt-get install -y \
    ros-melodic-rqt-multiplot
    
#Setup catkin workspace
RUN mkdir -p /root/apriltags_ws/src && \
    /bin/bash -c \
    "source /opt/ros/*/setup.bash && \
    cd /root/apriltags_ws && catkin init && \
    catkin config --extend /opt/ros/melodic --install -DCMAKE_BUILD_TYPE=Release"

#Install glxgears
RUN apt-get install -y \
    mesa-utils
    
#Install gnuplot
RUN apt-get install -y \ 
    gnuplot

