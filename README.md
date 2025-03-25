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
sudo apt-get install -y xauth x11-xserver-utils
sudo systemctl restart ssh
   ```
**Please restart the pi after the install**
## 2. Install docker compose:
```bash
# Download Docker Compose binary
sudo curl -L "https://github.com/docker/compose/releases/download/v2.23.3/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose

# Make it executable
sudo chmod +x /usr/local/bin/docker-compose

# Create a symbolic link
sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose

# Verify installation
docker-compose --version
   ```

## 2. Build docker contrainer
```bash
docker-compose build ros2-mission
   ```
## 3. Start the docker containter
```bash
docker-compose up -d
   ```
## 4. Connect to the docker container
```bash
docker-compose exec ros2-mission bash
   ```


## You can verify the USB port with this command and change it in docker-compose.yaml
```bash
ls -l /dev/ttyUSB*
   ```


## 5. Verify that ch341 driver is working:
```bash
ls -l /sys/class/tty/ttyUSB0/device/driver
```
You should see something like this:
```bash
/sys/class/tty/ttyUSB0/device/driver -> ../../../../../../../../../../bus/usb-serial/drivers/ch341-uart
```

## 6. Start the code!


Inside the container:

```bash

cd /ros2_ws
colcon build
source install/setup.bash
ros2 launch start_mission start.launch.py

ros2 run mission winch
```
## To use VNC for graphical interface:

Download this: https://www.realvnc.com/en/connect/download/viewer/

And use this command to run before launching ros2 script
```bash
export QT_QPA_PLATFORM=vnc
ros2 run mission control
```

```bash
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
```
```bash
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
```

Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow

Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultOutboundAction Allow

"networkingMode=mirrored"

