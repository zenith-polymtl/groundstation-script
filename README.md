# groundstation-script

How to use:

## 1. Install ROS2

You can install manually, or use the steps here:
https://github.com/zenith-polymtl/ardupilot_sim.git

## 2. Download the script
```bash
git clone 
```
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

