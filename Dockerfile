# base image for docker:
FROM osrf/ros:melodic-desktop
#FROM ubuntu:20.04

##########################################################################
# IF nvidia-container-runtime
# info: http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2
##########################################################################
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

##########################################################################
# Install common applications:
##########################################################################

ENV DEBIAN_FRONTEND=noninteractive 
RUN apt-get update && \
    apt-get install -y gnome-panel gnome-settings-daemon metacity nautilus gnome-terminal && \
    apt-get install -y tightvncserver && \
	apt-get install -y wget && \
	apt-get install -y tar python && \
	apt-get install -y nano python3-pip  

RUN apt-get update --fix-missing
RUN pip3 install --upgrade pip

### Only needed if running without ROS image:
## Install Python2
# RUN apt-get install -y python2-minimal
# Install pip2
RUN apt-get update 
RUN apt-get install -y curl 
WORKDIR /tmp
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
RUN python2 get-pip.py

### Only needed if running with ROS image:
# Install ROS tools:
RUN apt-get install -y ros-melodic-catkin
RUN pip3 install -U rosdep rosinstall_generator wstool rosinstall
# Add ROS to bashrc:
RUN bash -c "echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc"

##########################################################################
# Robot installations:
#
##########################################################################

# Install Pepper/NAO

## Install Choregraphe Suite 2.5.5.5
##
## Choregraphe key: 654e-4564-153c-6518-2f44-7562-206e-4c60-5f47-5f45
#-------------------------------------------------------------------------
RUN wget --no-check-certificate -P /opt/Softbank\ Robotics/ https://community-static.aldebaran.com/resources/2.5.10/Choregraphe/choregraphe-suite-2.5.10.7-linux64-setup.run
RUN chmod +x /opt/Softbank\ Robotics/choregraphe-suite-2.5.10.7-linux64-setup.run
RUN /opt/Softbank\ Robotics/choregraphe-suite-2.5.10.7-linux64-setup.run --mode unattended
RUN rm /opt/Softbank\ Robotics/choregraphe-suite-2.5.10.7-linux64-setup.run
RUN mv /opt/Softbank\ Robotics/Choregraphe\ Suite\ 2.5/lib/libz.so.1 libz.so.1.old
RUN ln -s /opt/Softbank\ Robotics/Choregraphe\ Suite\ 2.5/lib/x86_64-linux-gnu/libz.so.1
#-------------------------------------------------------------------------

# Install pynaoqi 2.5.5.5 library
#-------------------------------------------------------------------------
RUN wget --no-check-certificate -P /root/ https://community-static.aldebaran.com/resources/2.5.10/Python%20SDK/pynaoqi-python2.7-2.5.7.1-linux64.tar.gz
RUN tar -xvzf /root/pynaoqi-python2.7-2.5.7.1-linux64.tar.gz -C /root/
RUN rm /root/pynaoqi-python2.7-2.5.7.1-linux64.tar.gz
ENV PYTHONPATH /root/pynaoqi-python2.7-2.5.7.1-linux64/lib/python2.7/site-packages
ENV LD_LIBRARY_PATH /opt/Aldebaran/lib/
#-------------------------------------------------------------------------

##########################################################################
# Set up your environment:
#
## Uncomment block under the title of the tools you want:
##########################################################################

# Install requirements.txt
RUN apt-get install -y pkg-config libcairo2-dev libgirepository1.0-dev 
RUN apt-get install -y python-dev libffi-dev python3-dev
#COPY requirements.txt /opt/app/requirements.txt
#WORKDIR /opt/app
#RUN pip2 install -r requirements.txt
## Temp install opencv for pip2
RUN pip2 install opencv-python==4.2.0.32
RUN pip2 install configparser
RUN apt-get install -y iputils-ping vim

# Add directories from the host, to the guest:
## WORKDIR = Guest directory (within the docker):
# WORKDIR "/home/user/"   
## ADD [host_directory_path] [guest_directory_name]
# ADD test test/

# Set your final working dir
WORKDIR "/home/user/code"

# Auto build your ROS workspace:
# RUN bash -c '. /opt/ros/melodic/setup.bash; cd /home/user/ws; catkin_make'
# RUN bash -c "echo 'source /home/user/ws/devel/setup.bash' >> ~/.bashrc"
