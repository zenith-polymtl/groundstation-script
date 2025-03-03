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

# Install Python 3.10 and pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-venv \
    python3-dev \
    python3-pip \
    && python3 -m pip install --upgrade pip

RUN python3 -m pip install -U \
    colcon-common-extensions \
    vcstool \
    rosdep \
    pytest-cov \
    pytest-repeat \
    pytest-rerunfailures \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    setuptools==65.7.0

# Install ROS2 Humble (base version for better performance on Raspberry Pi)
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
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

# Create script to check for CH341 devices
RUN echo '#!/bin/bash\n\
echo "Checking for CH341 USB device..."\n\
if lsusb | grep -i "ch341"; then\n\
  echo "CH341 USB device found"\n\
else\n\
  echo "CH341 USB device not detected. Make sure it is connected."\n\
fi\n\
\n\
echo "Checking for serial devices..."\n\
if ls /dev/ttyUSB* 2>/dev/null; then\n\
  echo "Serial devices found. Your CH341 should be one of these."\n\
else\n\
  echo "No serial devices found. Please make sure:"\n\
  echo "1. The CH341 is connected"\n\
  echo "2. The ch341 module is loaded on the host (sudo modprobe ch341)"\n\
  echo "3. The container was started with --device=/dev/ttyUSB0 (or similar)"\n\
fi' > /usr/local/bin/check-devices && \
    chmod +x /usr/local/bin/check-devices

# Set up the entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Verify Python and pip versions
RUN python3 --version && pip --version

# Clone the repository
RUN git clone https://github.com/zenith-polymtl/ros2-mission-2 /ros2_ws

RUN pip3 install -r /ros2_ws/requirements.txt

# Set the default command to bash
CMD ["bash"]
