# FROM base image:latest => https://hub.docker.com/r/nvidia/cuda/ (ubuntu18.04, cuda 10.1-devel)
FROM ubuntu:latest
# RUN executes a shell command
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    && apt-get install -y software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && add-apt-repository ppa:ubuntu-toolchain-r/test \
    && apt-get update
# # Install gcc-9 and symbolink them
RUN apt-get install -y gcc-9
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90 --slave /usr/bin/g++ g++ \
    /usr/bin/g++-9 --slave /usr/bin/gcov gcov /usr/bin/gcov-9
RUN apt-get update && apt-get install -y \
    xz-utils \
    build-essential \
    curl
# Install ZeroMQ
RUN apt-get install -y libzmq3-dev
# Install Python3.7
RUN apt-get update && apt-get install -y python3.7
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2
# Install pip3
RUN apt-get install -y python3-pip
# Install Pytorch under pip3
RUN pip3 install torch torchvision
# Install Clang 9 (stable branch), just clang, lld, and lldb
# Download the latest version of Clang (official binary) for Ubuntu
# Extract the archive and add Clang to the PATH
RUN curl -sL http://releases.llvm.org/9.0.0/clang+llvm-9.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz \
    | tar -xJC . && \
    mv clang+llvm-9.0.0-x86_64-linux-gnu-ubuntu-18.04 clang_9.0.0 && \
    echo 'export PATH=/clang_9.0.0/bin:$PATH' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/clang_9.0.0/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

# Install cmake (3.15^)
RUN apt-get purge --auto-remove cmake && \
    curl -sL "https://cmake.org/files/v3.15/cmake-3.15.4-Linux-x86_64.tar.gz" \
    | tar --strip-components=1 -xz -C /usr/local && \
    apt-get update

# CMD defines the default command to be run in the container 
# CMD is overridden by supplying a command + arguments to 
# `docker run`, e.g. `nvcc --version` or `bash`
# Example CMD include:
# 1. `CMD bash` which starts an interactive shell (try to avoid this and write your CMD out below)
# 2. `CMD nvcc --version` which gets your Cuda version (10.1 as of Jul_28_19)