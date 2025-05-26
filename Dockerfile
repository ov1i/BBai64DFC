FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get install -y \
git build-essential diffstat \
chrpath socat cpio python3 python3-pip python3-pexpect \
xz-utils debianutils iputils-ping \
python3-git python3-jinja2 libegl1-mesa libsdl1.2-dev \
xterm sudo locales file gawk lz4 wget zstd tzdata

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN useradd -m -s /bin/bash ThomasTheTankEngine && echo 'ThomasTheTankEngine ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG sudo ThomasTheTankEngine

USER ThomasTheTankEngine
WORKDIR /home/ThomasTheTankEngine
