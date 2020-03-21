#!/bin/bash
rosbag record\
       -O $1\
       /skateboard/Acc\
       /skateboard/Gyro\
       /skateboard/Forces\
       /skateboard/Vel\
       /skateboard/Window
