FROM dustynv/ros:humble-ros-core-l4t-r32.7.1

# System dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    ros-humble-eigen3-cmake-module \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    apt-utils \
    python3-smbus \
    libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /MOLL-E

CMD ["/bin/bash"]
