FROM debian:stretch

MAINTAINER Gleb Golubitsky <sectoid@gnolltech.com>

# Non-interactive frontend for debian stuff to reduce error noise
ENV DEBIAN_FRONTEND noninteractive

# Install basic essentials
RUN apt -y update && \
    apt -y install openssh-client apt-utils curl wget zip sudo git make build-essential && \
    rm -rf /var/cache/apt/*

# Create lots of users so scp become usable
ADD docker/userhack.sh /
RUN /userhack.sh && rm -f /userhack.sh


ENV RUSTUP_HOME=/opt/rust \
    CARGO_HOME=/opt/rust

RUN ( curl https://sh.rustup.rs -sSf | sh -s -- -y --no-modify-path ) && \
    find /opt/rust -exec chmod 777 {} +

ADD docker/rust-wrapper.sh /usr/local/bin/cargo
ADD docker/rust-wrapper.sh /usr/local/bin/cargo-clippy
ADD docker/rust-wrapper.sh /usr/local/bin/cargo-fmt
ADD docker/rust-wrapper.sh /usr/local/bin/rls
ADD docker/rust-wrapper.sh /usr/local/bin/rust-gdb
ADD docker/rust-wrapper.sh /usr/local/bin/rust-lldb
ADD docker/rust-wrapper.sh /usr/local/bin/rustc
ADD docker/rust-wrapper.sh /usr/local/bin/rustdoc
ADD docker/rust-wrapper.sh /usr/local/bin/rustfmt
ADD docker/rust-wrapper.sh /usr/local/bin/rustup
