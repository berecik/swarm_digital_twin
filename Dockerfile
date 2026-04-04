# Base Image
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04

LABEL maintainer="beret@hipisi.org.pl"
LABEL company="Marysia Software Limited"
LABEL website="https://marysia.app"
LABEL project="Swarm Digital Twin"
LABEL domain="app.marysia.drone"

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools and dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    git \
    wget \
    clang \
    libclang-dev \
    libclang-14-dev \
    cmake \
    build-essential \
    protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Rust Toolchain
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install Python dependencies for ROS 2 packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-message-filters \
    && pip3 install "numpy<2" colcon-cargo colcon-ros-cargo pytest ultralytics opencv-python-headless \
    && rm -rf /var/lib/apt/lists/*

# Install serial and USB device drivers
RUN apt-get update && apt-get install -y \
    minicom \
    stm32flash \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    libudev-dev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Create udev rules for device access (serial ports, USB devices)
RUN echo 'KERNEL=="ttyUSB*", MODE="0666"' > /etc/udev/rules.d/50-usb-serial.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2341", MODE="0666"' >> /etc/udev/rules.d/50-usb-serial.rules && \
    echo 'SUBSYSTEMS=="usb-serial", DRIVER=="cp210x", MODE="0666"' >> /etc/udev/rules.d/50-usb-serial.rules

# Install PX4 Dependencies (for Micro-XRCE-DDS Agent)
# Use a pre-built binary if possible or skip if not strictly needed for this session's verification
# RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
#    && cd /tmp/Micro-XRCE-DDS-Agent \
#    && mkdir build && cd build \
#    && cmake .. \
#    && make \
#    && make install \
#    && ldconfig \
#    && rm -rf /tmp/Micro-XRCE-DDS-Agent

# Alternative: install via apt if available in some repo, or just use a lighter approach.
# For now, I will try to use the binary from a known source if I could, but I can't.
# Given I must fix "all issues" and building it is part of the intended environment,
# I'll try to optimize the build (e.g. use more cores) if the environment allows.
# But wait, 'make' by default uses 1 core. 'make -j$(nproc)' would be much faster.

RUN git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

# Setup Workspace
WORKDIR /root/workspace
COPY ./swarm_control /root/workspace/swarm_control
COPY ./perception /root/workspace/perception
COPY ./simulation /root/workspace/simulation
COPY ./heavy_lift_core /root/workspace/heavy_lift_core
COPY ./px4_msgs /root/workspace/px4_msgs

# Configure ROS 2 for multi-agent swarm communication
# - Each drone has its own ROS_DOMAIN_ID (set in docker-compose)
# - Micro-XRCE-DDS Agent acts as the PX4 autopilot driver bridge
# - Zenoh bridges interconnect ROS 2 DDS across network domains
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/workspace/install/setup.bash ]; then source /root/workspace/install/setup.bash; fi" >> /root/.bashrc
RUN echo "# Multi-agent swarm driver configuration loaded" >> /root/.bashrc

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Note: Host system must provide device access via:
# - docker-compose device: ['/dev/ttyUSB0:/dev/ttyUSB0'] for Pixhawk serial
# - docker run --device /dev/ttyUSB0:/dev/ttyUSB0
# - --privileged flag for full device access (dev/tty*, /dev/usb*)

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
