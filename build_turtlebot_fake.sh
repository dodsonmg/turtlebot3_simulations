#!/bin/bash

IN=/home/md403/rosserial/rosserial_turtlebot_ubuntu/turtlebot3_simulations/turtlebot3_fake/src/turtlebot3_fake_rosserial.cpp
OUT=turtlebot3_fake_rosserial

g++ $IN -o $OUT \
	-g \
	-I/auto/homes/md403/rosserial/rosserial_turtlebot_ubuntu/rosserial_cheribsd/ros_lib \
	-I/auto/homes/md403/rosserial/rosserial_turtlebot_ubuntu/turtlebot3_simulations/turtlebot3_fake/include \
	-I/auto/homes/md403/rosserial/rosserial_turtlebot_ubuntu/turtlebot3_simulations/turtlebot3_fake/include/turtlebot3_fake
