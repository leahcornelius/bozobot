# BozoBot
A ROS 2 package for my robot, BozoBot. The bozobot package runs on a raspberry pi 4 on the robot, designed to run on Ubuntu 20.04 64 bit with ROS 2 Foxy. To be able to use a picamera with rpi ubuntu, i had to modify /boot/firmware/config.txt (using usercfg.txt didn't work) and add the following lines:
```bash
start_x=1
gpu_mem=128
```
The picamera should now be available as /dev/video0 (or /dev/video1) and can be used like a usual V4L camera, i use RosUsbCam to publish the camera feed to ROS and cv_bridge to convert the ROS image frames into open cv BGR images for processing. 

## Build
# Linux
Assumming ROS 2 Foxy is installed, clone this repo into the src folder of your workspace and build the package using colcon: (replace ~/ros2_ws with the path to your workspace)
```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/leocornelius/bozobot.git
cd ~/ros2_ws
colcon build --symlink-install
```

# Non-Linux
No clue, should probably work with enough vodka and patience but i value my sanity too much to fuck around with windows. Dangerous shit. GLHF if you try, let me know how it goes in the issues.

# Run
Open another terminal and source both the ROS 2 setup script and the overlay/ local setup script:
```bash
source /opt/ros/foxy/setup.bash # Only needed if you haven't already sourced it/ is not in your .bashrc
cd ~/ros2_ws/
source install/setup.bash
```

Then run the launch file:
```bash
ros2 launch bozobot robot.launch.py
```


