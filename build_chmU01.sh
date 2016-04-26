#!/bin/bash

LOCAL_DIR=`pwd`
export PATH=$PATH:$LOCAL_DIR/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin

make ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- hisi_hi6210sft_defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- -j$1 
