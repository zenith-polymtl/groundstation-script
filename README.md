# dockerfile-raspberrypi5

How to use:

## 1. Install docker & other requirements
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y docker.io
sudo apt-get install -y raspberrypi-kernel-headers
sudo modprobe ch341
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
   ```
**Please restart the pi after the install**

## 2. Build docker contrainer
```bash
docker build -t ros2_humble_pi .
   ```
## 3. Check which USB port
```bash
ls -l /dev/ttyUSB*
   ```

## 4. Start the docker containter
```bash
docker run -it --rm --privileged --device=/dev/ttyUSB0 --network=host ros2_humble_pi
   ```

-it: Runs interactively with a terminal

--rm: Automatically removes the container when you exit

--network=host: Uses the host's network, which is important for ROS2 communication

ros2_humble_pi: The name of the image we built

--privileged: Makes sure it can access USB devices

--device=/dev/ttyUSB0: Connects to USB0

## 5. Working with Your Own ROS2 Code and Mount Directory
If you want to develop and run your own ROS2 code:

Mount Your Code Directory
Exit both containers (CTRL+C to stop the nodes, then type exit), then start a new container with your code directory mounted:

```bash

docker run -it --rm --privileged --device=/dev/ttyUSB0 --network=host -v /path/to/your/ros2_ws:/ros2_ws ros2_humble_pi
```
Replace /path/to/your/ros2_ws with the actual path to your ROS2 workspace on your Raspberry Pi.

Build and Run Your Code

Inside the container:

```bash

cd /ros2_ws
colcon build
source install/setup.bash
ros2 run your_package your_node
```
