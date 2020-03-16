#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Default to building for ubuntu"
	TARGET=ubuntu
else
	TARGET=$1
fi

IN_PHYSICAL=./turtlebot3_rosserial/src/turtlebot3_rosserial_physical.cpp
IN_COMMS=./turtlebot3_rosserial/src/turtlebot3_rosserial_comms.cpp
TIME=../rosserial_cheribsd/ros_lib/time.cpp
DURATION=../rosserial_cheribsd/ros_lib/duration.cpp
EMBEDDED_LINUX_COMMS=../rosserial_cheribsd/ros_lib/embedded_linux_comms.c

OUT=turtlebot3_rosserial_$TARGET

if [[ "$TARGET" == "ubuntu" ]]; then
	g++ $IN_PHYSICAL $IN_COMMS $TIME $DURATION $EMBEDDED_LINUX_COMMS -o $OUT \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		-DBUILD_LIBROSSERIALEMBEDDEDLINUX
elif [[ "$TARGET" == "cheri" ]]; then
	$HOME/cheri/output/sdk/bin/cheri-unknown-freebsd-clang++ \
		$IN_PHYSICAL $IN_COMMS $TIME $DURATION $EMBEDDED_LINUX_COMMS -o $OUT \
		-mabi=purecap \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		-DBUILD_LIBROSSERIALEMBEDDEDLINUX \
		--sysroot=$HOME/cheri/output/sdk/sysroot128 \
		-B$HOME/cheri/output/sdk
fi



