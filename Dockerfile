# Use official Ubuntu 22.04 base image
FROM ubuntu:22.04

#RMW ZENOH experimental flag
ARG EXPERIMENTAL_ZENOH_RMW=FALSE

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# Env setup
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    git \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# Install dependencies
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    wget \ 
    software-properties-common \ 
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    libpython3-dev \
    python3-tk \
    python3-typing-extensions \
    python3-pytest-cov \
    python3-protobuf \
    ros-humble-desktop \
    ros-dev-tools \
    # RTAB-Map packages
    ros-humble-rtabmap-ros \
    ros-humble-octomap-server \
    # Controller packages
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-controller-interface \
    ros-humble-hardware-interface \
    ros-humble-forward-command-controller \
    #check if Zenoh should be installed
    $(if [ "$EXPERIMENTAL_ZENOH_RMW" = "TRUE" ]; then echo "ros-humble-rmw-zenoh-cpp"; fi) \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros_ws/src

# Initialize rosdep (only if not already initialized)
RUN if [ ! -d "/etc/ros/rosdep/sources.list.d" ] || [ -z "$(ls -A /etc/ros/rosdep/sources.list.d/*.list 2>/dev/null)" ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Clone driver code
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git .
RUN git submodule update --init --recursive

# Install additional dependencies that might be missing
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-dev \
    python3-numpy \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Source ROS and install dependencies with rosdep (ignore failures for now)
RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths . --ignore-src -y -r --rosdistro=humble || true

# Run install script and pass in the architecture
RUN ARCH=$(dpkg --print-architecture) && echo "Building driver with $ARCH" && /ros_ws/src/install_spot_ros2.sh --$ARCH

# Build packages with Colcon
WORKDIR /ros_ws/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Set up workspace directory
RUN mkdir -p /home/spot_ws

# Set working directory
WORKDIR /home/spot_ws

# Set environment variables for ROS Domain ID and other configs
ENV HOME=/home/spot_ws
ENV USER=root
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/spot_ws/fastdds_profile.xml

# Create bashrc with ROS setup
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /ros_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc

# Set default command
CMD ["/bin/bash"]
