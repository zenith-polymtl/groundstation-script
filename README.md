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

## 6. Verify that ch341 driver is working:
```bash
ls -l /sys/class/tty/ttyUSB0/device/driver
```
You should see something like this:
```bash
/sys/class/tty/ttyUSB0/device/driver -> ../../../../../../../../../../bus/usb-serial/drivers/ch341-uart
```

## 5. Start the code!


Inside the container:

```bash

cd /ros2_ws
colcon build
source install/setup.bash
ros2 launch start_mission start.launch.py
```
