#!/bin/bash

TARGET=$1

IN=./turtlebot3_fake/src/turtlebot3_fake_rosserial.cpp
OUT=turtlebot3_fake_rosserial

if [[ "$TARGET" == "ubuntu" ]]; then
	g++ $IN -o $OUT \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_fake/include \
		-I./turtlebot3_fake/include/turtlebot3_fake
elif [[ "$TARGET" == "cheri" ]]; then
	$HOME/cheri/output/sdk/bin/cheri-unknown-freebsd-clang++ $IN -o $OUT \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_fake/include \
		-I./turtlebot3_fake/include/turtlebot3_fake \
		--sysroot=$HOME/cheri/output/sdk/sysroot128 \
		-B$HOME/cheri/output/sdk
fi



