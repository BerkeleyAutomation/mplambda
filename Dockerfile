# FROM base image:latest => https://hub.docker.com/r/nvidia/cuda/ (ubuntu18.04, cuda 10.1-devel)
FROM ubuntu:latest
# RUN executes a shell command
# software-properties-common has the command add-apt-repository in it
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    && apt-get install -y software-properties-common \
    && add-apt-repository -y ppa:deadsnakes/ppa \
    && add-apt-repository -y ppa:ubuntu-toolchain-r/test \
    && apt-get update
# Install gcc-9, zeroMQ (libzmq3-dev), python3, curl
RUN apt install -y build-essential subversion python3-dev \
        libncurses5-dev libxml2-dev libedit-dev swig doxygen graphviz xz-utils \
        gcc-9 g++-9 python3-pip curl libzmq3-dev git

# gcc and g++ symbolinks 
RUN rm /usr/bin/gcc && \
    rm /usr/bin/g++ && \
    ln -s /usr/bin/gcc-9 /usr/bin/gcc && \
    ln -s /usr/bin/g++-9 /usr/bin/g++
    
# Install Pytorch under pip3
RUN pip3 install torch torchvision

# Install Clang 9 (stable branch), just clang, lld, and lldb
# Install cmake (3.15^)
RUN apt-get purge --auto-remove cmake && \
    curl -sL "https://cmake.org/files/v3.15/cmake-3.15.4-Linux-x86_64.tar.gz" \
    | tar --strip-components=1 -xz -C /usr/local 

RUN curl -sL http://releases.llvm.org/9.0.0/clang+llvm-9.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz \
    | tar -xJC . && mv clang+llvm-9.0.0-x86_64-linux-gnu-ubuntu-18.04 /usr/local/clang_9.0.0 && \
    echo 'export PATH=/usr/local/clang_9.0.0/bin:$PATH' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/usr/local/clang_9.0.0/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    mkdir serverless && \
    cd serverless && \
    git clone https://github.com/UNC-Robotics/nigh.git && \
    git clone https://github.com/hydro-project/anna.git && \
    curl -sL "http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz" \
    | tar --strip-components=1 -xz -C /serverless && \ 
    git clone https://github.com/danfis/libccd.git && \
    cd libccd && mkdir build && cd build && \
    cmake -G "Unix Makefiles" .. && \
    make -j8 && make install && \
    cd /serverless && \
    git clone https://github.com/flexible-collision-library/fcl.git && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && make -j8 && make install &&\
    cd /serverless && \
    git clone https://github.com/assimp/assimp.git && \
    apt update && \
    apt install -y libassimp-dev libprotobuf10 \
                    libcurl4-openssl-dev libssl-dev \
                    unzip wget       

# Protobuf install and Anna cmake build
RUN git submodule init && git submodule update && \
    wget https://github.com/google/protobuf/releases/download/v3.5.1/protobuf-all-3.5.1.zip && \
    unzip protobuf-all-3.5.1.zip && \
    cd protobuf-3.5.1 && \
    ./autogen.sh && \
    ./configure CXX=g++ CXXFLAGS='-std=c++11 -O3 -g' && \
    make -j4 && make install && \
    cd /serverless/anna/ && \
    rm -rf build && mkdir build && cd build && \
    cmake -std=c++11 "-GUnix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE-CXX_COMPILER="/usr/bin/g++" .. && \
    make -j8 && make install

# First installs aws-lambda-cpp runtime, and then the aws-sdk-cpp CLI
# Ref: https://aws.amazon.com/blogs/compute/introducing-the-c-lambda-runtime/
# RUN cd /serverless && \
 #   git clone https://github.com/awslabs/aws-lambda-cpp.git && \
 #   cd aws-lambda-cpp && \
 #   mkdir build && \
 #   cd build && \
 #   cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
 #   -DCMAKE_INSTALL_PREFIX=~/out && \
 #   make && make install && \
 #   cd /serverless && \
 #   git clone https://github.com/aws/aws-sdk-cpp.git && \
 #   cd aws-sdk-cpp && \
 #   mkdir build && cd build && \
 #   cmake .. -DBUILD_ONLY=s3 \
 #   -DBUILD_SHARED_LIBS=OFF \
 #   -DENABLE_UNITY_BUILD=ON \
 #   -DCMAKE_BUILD_TYPE=Release \
 #   -DCMAKE_INSTALL_PREFIX=~/out


# https://solarianprogrammer.com/2013/01/17/building-clang-libcpp-ubuntu-linux/
