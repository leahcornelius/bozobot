# BozoBot
A ROS 2 package for my robot, BozoBot. The bozobot package runs on a raspberry pi 4 on the robot, designed to run on Ubuntu 20.04 64 bit with ROS 2 Foxy. To be able to use a picamera with rpi ubuntu, i had to modify /boot/firmware/config.txt (using usercfg.txt didn't work) and add the following lines:
```bash
start_x=1
gpu_mem=128
```
The picamera should now be available as /dev/video0 (or /dev/video1) and can be used like a usual V4L camera, i use RosUsbCam to publish the camera feed to ROS and cv_bridge to convert the ROS image frames into open cv BGR images for processing. 

