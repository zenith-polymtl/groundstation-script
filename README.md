# dockerfile-raspberrypi5

How to use:

1. Install docker
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
   ```
**Please restart the pi after the install**

2. Build docker contrainer
```bash
docker build -t ros2_humble_pi .
   ```
3. Start
```bash
docker run -it --rm --network=host ros2_humble_pi
   ```

-it: Runs interactively with a terminal

--rm: Automatically removes the container when you exit

--network=host: Uses the host's network, which is important for ROS2 communication

ros2_humble_pi: The name of the image we built
