FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set locale as per official documentation
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install dependencies for adding ROS2 repository, Python 3.11, and build tools
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    git \
    build-essential \
    gcc \
    unzip \
    linux-modules-extra-$(uname -r) \
    usbutils \
    kmod

# Add Universe repository
RUN add-apt-repository universe

# Add deadsnakes PPA for Python 3.11
RUN add-apt-repository ppa:deadsnakes/ppa

# Add ROS2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and upgrade to avoid systemd/udev issues mentioned in the warning
RUN apt-get update && apt-get upgrade -y

# Install Python 3.11 and pip
RUN apt-get update && apt-get install -y \
    python3.11 \
    python3.11-venv \
    python3.11-dev \
    python3-pip \
    && ln -sf /usr/bin/python3.11 /usr/bin/python3 \
    && ln -sf /usr/bin/python3.11 /usr/bin/python \
    && curl -sS https://bootstrap.pypa.io/get-pip.py | python3.11 \
    && python3 -m pip install --upgrade pip

# Install ROS2 Humble (base version for better performance on Raspberry Pi)
# You can change to ros-humble-desktop if you need GUI tools and have sufficient resources
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install canusb module
RUN mkdir -p /opt/canusb && \
    cd /opt/canusb && \
    git clone https://github.com/kobolt/usb-can . && \
    gcc -o canusb canusb.c && \
    # Add to PATH
    ln -sf /opt/canusb/canusb /usr/local/bin/canusb

# Install CH341 driver (Note: This requires privileged container to work)
RUN mkdir -p /opt/ch341 && \
    cd /opt/ch341 && \
    git clone https://github.com/SeeedDocument/USB-CAN-Analyzer.git && \
    cd USB-CAN-Analyzer/res/Driver && \
    unzip CH341SER_LINUX.ZIP && \
    cd CH341SER_LINUX && \
    make

# Add a script to load the CH341 driver when needed
RUN echo '#!/bin/bash\n\
if [ -f /opt/ch341/USB-CAN-Analyzer/res/Driver/CH341SER_LINUX/ch34x.ko ]; then\n\
  insmod /opt/ch341/USB-CAN-Analyzer/res/Driver/CH341SER_LINUX/ch34x.ko\n\
  echo "CH341 driver loaded"\n\
else\n\
  echo "CH341 driver not found"\n\
fi' > /usr/local/bin/load-ch341 && \
chmod +x /usr/local/bin/load-ch341

# Set up the entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Verify Python and pip versions
RUN python3 --version && pip --version

# Clone the repository
RUN git clone https://github.com/zenith-polymtl/ros2-mission-2 /ros2_ws/src/ros2-mission-2

# Set the default command to bash
CMD ["bash"]
