FROM dustynv/ros:humble-ros-core-l4t-r32.7.1

# System dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    apt-utils \
    python3-smbus \
    libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /MOLL-E

# Launch file or entrypoint
COPY new_entrypoint.sh /usr/local/bin/new_entrypoint.sh
RUN chmod +x /usr/local/bin/new_entrypoint.sh

# Set new entrypoint
ENTRYPOINT ["/usr/local/bin/new_entrypoint.sh"]
CMD ["/bin/bash"]
