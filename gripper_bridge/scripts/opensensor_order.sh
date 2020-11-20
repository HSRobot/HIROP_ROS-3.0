#!/bin/bash

sudo su
cd /home/de/catkin_ws/src/HsDualAppBridge/rb_ui/build/bin
source /opt/ros/kinetic/setup.bash
./daq_ros_driver

wait
exit 0



