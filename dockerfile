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
    pymavlink \
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


# Add this before the pip install command:
RUN apt-get update && apt-get install -y \
  qt6-base-dev \
  qt6-declarative-dev \
  libgl1-mesa-dev \
  libxkbcommon-x11-0 \
  libxcb-cursor0 \
  libxcb-icccm4 \
  libxcb-keysyms1 \
  libxcb-shape0 \
  x11-apps \ 
  x11-utils \
  xauth \
  && rm -rf /var/lib/apt/lists/*
RUN pip3 install --only-binary=PyQt6 PyQt6

RUN apt-get update && apt-get install -y \
  libxcb-cursor0 \
  libxcb-xinerama0 \
  libxcb-randr0 \
  libxcb-icccm4 \
  libxcb-keysyms1 \
  libxcb-shape0 \
  libxcb-render-util0 \
  libxcb-xkb1 \
  libxkbcommon-x11-0


# Set up the entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Verify Python and pip versions
RUN python3 --version && pip --version

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set the default command to bash
CMD ["bash"]
