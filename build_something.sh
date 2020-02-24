#!/bin/bash

IN=$1
OUT=$2

g++ $IN -o $OUT \
	-g \
	-I/auto/homes/md403/rosserial/rosserial_turtlebot_ubuntu/rosserial_cherbsd/ros_lib \
	-I/auto/homes/md403/rosserial/rosserial_turtlebot_ubuntu/turtlebot3_simulations/turtlebot3_fake/include \
	-I/auto/homes/md403/rosserial/rosserial_turtlebot_ubuntu/turtlebot3_simulations/turtlebot3_fake/include/turtlebot3_fake
