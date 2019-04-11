FROM ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y
RUN apt-get install software-properties-common -y
RUN add-apt-repository ppa:george-edison55/cmake-3.x -y
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y

RUN \
  sed -i 's/# \(.*multiverse$\)/\1/g' /etc/apt/sources.list && \
  apt-get update && \
  apt-get -y upgrade && \
  apt-get install -y build-essential && \
  apt-get install -y software-properties-common && \
  apt-get install -y \
	apt-transport-https \
	byobu \ 
	curl \ 
	git \
	htop \
	man \ 
	unzip \
	vim \
	wget \
	libssl-dev \
	libusb-1.0-0-dev \ 
	pkg-config \
	libgtk-3-dev \ 
	v4l-utils \
	cmake \
	freeglut3-dev \ 
	libblas-dev \
	libboost-all-dev \ 
	libgl1-mesa-dev && \
	apt-get clean && rm -rf /var/lib/apt/lists/* && \
	echo '\n\n\n' | ssh-keygen -t rsa

# Run installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/install_gcc.sh
RUN bash /tmp/installers/install_glfw3.sh
#RUN bash /tmp/installers/install_realsense.sh
RUN bash /tmp/installers/install_glew.sh
RUN bash /tmp/installers/post_install.sh

WORKDIR /apollo
RUN useradd -ms /bin/bash twizy1
