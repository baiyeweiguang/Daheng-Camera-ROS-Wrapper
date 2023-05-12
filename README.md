# Daheng Galaxy Camera Driver

## Introduction

This is a ros wrapper of Daheng Galaxy Camera

Dependencies:
- ROS Noetic
- gxiapi

## Get started
1. Put the code in your catkin workspace, then
```
catkin_make
```
2. Connect the Daheng Camera with your computer, then
```
source devel/setup.bash
roslaunch daheng_galaxy camera.launch
```

## Acknowledgement

This repository achieves a simple way to run Daheng camera in ROS. 

There is a more complete version of Daheng Galaxy Camera ros wrapper. See https://github.com/qiayuanliao/galaxy_camera