#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Default to building for ubuntu"
	TARGET=ubuntu
else
	TARGET=$1
fi

# library names and locations for physical and comms .so files
OUT_PHYSICAL_NAME=turtlebot3_rosserial_physical_$TARGET
OUT_COMMS_NAME=turtlebot3_rosserial_comms_$TARGET
LIB_LOCATION=./turtlebot3_rosserial/lib

# names for input and output of main source file
IN_MAIN=./turtlebot3_rosserial/src/turtlebot3_rosserial.cpp
OUT_MAIN=./turtlebot3_rosserial/bin/turtlebot3_rosserial_$TARGET

# names for input support source files
TIME=../rosserial_cheribsd/ros_lib/time.cpp
DURATION=../rosserial_cheribsd/ros_lib/duration.cpp
EMBEDDED_LINUX_COMMS=../rosserial_cheribsd/ros_lib/embedded_linux_comms.c

if [[ "$TARGET" == "ubuntu" ]]; then
	g++ $IN_MAIN -o $OUT_MAIN \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		-L$LIB_LOCATION \
		-l$OUT_PHYSICAL_NAME \
		-l$OUT_COMMS_NAME \
		-DBUILD_LIBROSSERIALEMBEDDEDLINUX
elif [[ "$TARGET" == "cheri" ]]; then
	$HOME/cheri/output/sdk/bin/cheri-unknown-freebsd-clang++ \
		$IN_MAIN -o $OUT_MAIN \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		-L$LIB_LOCATION \
		-l$OUT_PHYSICAL_NAME \
		-l$OUT_COMMS_NAME \
		-DBUILD_LIBROSSERIALEMBEDDEDLINUX \
		-mabi=purecap \
		--sysroot=$HOME/cheri/output/sdk/sysroot128 \
		-B$HOME/cheri/output/sdk
fi



