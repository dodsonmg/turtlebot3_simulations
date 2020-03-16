#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Default to building for ubuntu"
	TARGET=ubuntu
else
	TARGET=$1
fi

IN_PHYSICAL=./turtlebot3_rosserial/src/turtlebot3_rosserial_physical.cpp
OUT_PHYSICAL=./turtlebot3_rosserial/lib/libturtlebot3_rosserial_physical_$TARGET.so
OUT_PHYSICAL_NAME=turtlebot3_rosserial_physical_$TARGET

IN_COMMS=./turtlebot3_rosserial/src/turtlebot3_rosserial_comms.cpp
OUT_COMMS=./turtlebot3_rosserial/lib/libturtlebot3_rosserial_comms_$TARGET.so
OUT_COMMS_NAME=turtlebot3_rosserial_comms_$TARGET

TIME=../rosserial_cheribsd/ros_lib/time.cpp
DURATION=../rosserial_cheribsd/ros_lib/duration.cpp
EMBEDDED_LINUX_COMMS=../rosserial_cheribsd/ros_lib/embedded_linux_comms.c

if [[ "$TARGET" == "ubuntu" ]]; then
	# build shared library from turtlebot3_rosserial_physical
	g++ -fPIC -shared $IN_PHYSICAL $TIME $DURATION $EMBEDDED_LINUX_COMMS -o $OUT_PHYSICAL \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		-DBUILD_LIBROSSERIALEMBEDDEDLINUX

	# build shared library from turtlebot3_rosserial_comms
	g++ -fPIC -shared $IN_COMMS $TIME $DURATION $EMBEDDED_LINUX_COMMS -o $OUT_COMMS \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		-DBUILD_LIBROSSERIALEMBEDDEDLINUX
elif [[ "$TARGET" == "cheri" ]]; then
	# build shared library from turtlebot3_rosserial_physical
	$HOME/cheri/output/sdk/bin/cheri-unknown-freebsd-clang++ \
		-fPIC -shared $IN_PHYSICAL $TIME $DURATION $EMBEDDED_LINUX_COMMS -o $OUT_PHYSICAL \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		-DBUILD_LIBROSSERIALEMBEDDEDLINUX \
		-mabi=purecap \
		--sysroot=$HOME/cheri/output/sdk/sysroot128 \
		-B$HOME/cheri/output/sdk

	# build shared library from turtlebot3_rosserial_comms
	$HOME/cheri/output/sdk/bin/cheri-unknown-freebsd-clang++ \
		-fPIC -shared $IN_COMMS $TIME $DURATION $EMBEDDED_LINUX_COMMS -o $OUT_COMMS \
		-g \
		-I../rosserial_cheribsd/ros_lib \
		-I./turtlebot3_rosserial/include \
		-I./turtlebot3_rosserial/include/turtlebot3_rosserial \
		-DBUILD_LIBROSSERIALEMBEDDEDLINUX \
		-mabi=purecap \
		--sysroot=$HOME/cheri/output/sdk/sysroot128 \
		-B$HOME/cheri/output/sdk
fi



