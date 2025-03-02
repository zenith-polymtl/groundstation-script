FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set locale as per official documentation
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install dependencies for adding ROS2 repository, Python 3.11, and build tools
# Added linux-headers-generic to provide kernel headers
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    git \
    build-essential \
    gcc \
    linux-headers-generic \
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

# Create a script to build and load the CH341 driver at runtime
# This is more reliable than trying to build during image creation
RUN mkdir -p /opt/ch341 && \
    cd /opt/ch341 && \
    git clone https://github.com/SeeedDocument/USB-CAN-Analyzer.git && \
    cd USB-CAN-Analyzer/res/Driver && \
    unzip CH341SER_LINUX.ZIP && \
    cd CH341SER_LINUX && \
    # Create a script to build and load the module at runtime
    echo '#!/bin/bash\n\
echo "Building CH341 driver for kernel $(uname -r)"\n\
cd /opt/ch341/USB-CAN-Analyzer/res/Driver/CH341SER_LINUX\n\
rm -f *.ko *.o *.mod.* Module.*\n\
make clean\n\
# Try to install specific kernel headers if generic ones don't work\n\
if [ ! -d "/lib/modules/$(uname -r)/build" ]; then\n\
  echo "Kernel headers not found, attempting to install..."\n\
  apt-get update && apt-get install -y linux-headers-$(uname -r) || true\n\
fi\n\
# If headers still not available, try to use the generic ones\n\
if [ ! -d "/lib/modules/$(uname -r)/build" ] && [ -d "/usr/src/linux-headers-generic" ]; then\n\
  echo "Using generic headers"\n\
  ln -sf /usr/src/linux-headers-generic /lib/modules/$(uname -r)/build\n\
fi\n\
# Attempt to build\n\
make\n\
if [ -f "ch34x.ko" ]; then\n\
  echo "CH341 driver compiled successfully"\n\
  # Load the module if running in privileged mode\n\
  if [ -w "/sys/module" ]; then\n\
    insmod ch34x.ko && echo "CH341 driver loaded successfully"\n\
  else\n\
    echo "Container not running in privileged mode, cannot load module"\n\
  fi\n\
else\n\
  echo "CH341 driver compilation failed"\n\
fi' > /usr/local/bin/build-ch341-driver && \
    chmod +x /usr/local/bin/build-ch341-driver

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
