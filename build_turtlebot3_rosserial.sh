#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Default to building for ubuntu"
	TARGET=ubuntu
else
	TARGET=$1
fi

IN=./turtlebot3_rosserial/src/turtlebot3_rosserial.cpp
OUT=turtlebot3_rosserial_$TARGET

if [[ "$TARGET" == "ubuntu" ]]; then
	g++ $IN -o $OUT \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial
elif [[ "$TARGET" == "cheri" ]]; then
	$HOME/cheri/output/sdk/bin/cheri-unknown-freebsd-clang++ $IN -o $OUT \
		-mabi=purecap \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		--sysroot=$HOME/cheri/output/sdk/sysroot128 \
		-B$HOME/cheri/output/sdk
	
	cp $OUT $HOME/cheri/output/rootfs128/xfer
fi



