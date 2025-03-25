# groundstation-script

How to use:

## 0. Enable networking for WSL2 on Windows
Modify or create this file:
```powershell
    %UserProfile%\.wslconfig
```
The file must contain:
```
[wsl2]
"networkingMode=mirrored"
```

## 1. Install ROS2

You can install manually, or use the steps here:
https://github.com/zenith-polymtl/ardupilot_sim.git

## 2. Download the script
Install dependencies:
```bash
sudo apt-get update && sudo apt-get install -y  ros-humble-rmw-cyclonedds-cpp ros-humble-demo-nodes-cpp qt6-base-dev qt6-declarative-dev libgl1-mesa-dev libxkbcommon-x11-0 libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 libxcb-shape0 x11-apps x11-utils xauth x11vnc xvfb fluxbox
```


## 3. Download the script
```bash
git clone https://github.com/zenith-polymtl/groundstation-script.git
cd groundstation-script
```

## 4. Enable script:
```bash
chmod +x ./start_vnc.sh
```
## 5. Run script script:
```bash
./start_vnc.sh
```

## To use VNC for graphical interface:

Download this: https://www.realvnc.com/en/connect/download/viewer/

## Test connectivity with:
```bash
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
```
```bash
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
```

Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow

Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultOutboundAction Allow



