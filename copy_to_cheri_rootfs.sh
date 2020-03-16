#!/bin/bash

TARGET=cheri

# library names and locations for physical and comms .so files
LIB_LOCATION=./turtlebot3_rosserial/lib
OUT_PHYSICAL_NAME=$LIB_LOCATION/libturtlebot3_rosserial_physical_$TARGET.so
OUT_COMMS_NAME=$LIB_LOCATION/libturtlebot3_rosserial_comms_$TARGET.so

# names for main executable and run script
BIN_LOCATION=./turtlebot3_rosserial/bin
OUT_MAIN=$BIN_LOCATION/turtlebot3_rosserial_$TARGET
RUN_SCRIPT=$BIN_LOCATION/run_cheri.sh

# name for the run script


cp $OUT_PHYSICAL_NAME $HOME/cheri/output/rootfs128/xfer
cp $OUT_COMMS_NAME $HOME/cheri/output/rootfs128/xfer
cp $OUT_MAIN $HOME/cheri/output/rootfs128/xfer
cp $RUN_SCRIPT $HOME/cheri/output/rootfs128/xfer