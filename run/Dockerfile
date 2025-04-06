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
    # vcs \ 
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /MOLL-E

# Build external packages: tf2_sensor_msgs and slam_toolbox
# This step uses rosinstall_generator to create a rosinstall file for the two packages
# then uses vcs to import them into src/external, installs any dependencies via rosdep,
# builds them with colcon, and finally cleans up the source.
RUN source /opt/ros/humble/install/setup.bash && \
    mkdir -p src/external && \
    rosinstall_generator --deps --rosdistro humble tf2_sensor_msgs slam_toolbox > ros2.humble.external.rosinstall && \
    cat ros2.humble.external.rosinstall && \
    vcs import src/external < ros2.humble.external.rosinstall && \
    apt-get update && \
    rosdep install -y --ignore-src --from-paths src/external --rosdistro humble && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --merge-install --base-paths src/external && \
    rm -rf src/external

CMD ["/bin/bash"]
