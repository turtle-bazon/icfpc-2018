FROM debian:stretch

MAINTAINER Gleb Golubitsky <sectoid@gnolltech.com>

# Non-interactive frontend for debian stuff to reduce error noise
ENV DEBIAN_FRONTEND noninteractive

# Install basic essentials
RUN apt -y update && \
    apt -y install openssh-client apt-utils curl wget zip sudo git make && \
    rm -rf /var/cache/apt/*

# Create lots of users so scp become usable
ADD docker/userhack.sh /
RUN /userhack.sh && rm -f /userhack.sh
