#!/bin/bash

DIR=`pwd`
echo $DIR

./configure --prefix=$(DIR)/mp4v2-2.0.0/lib-ax --host=arm-linux \
    CC=/usr/local/ARM-toolchain/arm-AX620E-linux-uclibcgnueabihf/bin/arm-AX620E-linux-uclibcgnueabihf-gcc  CXX=/usr/local/ARM-toolchain/arm-AX620E-linux-uclibcgnueabihf/bin/arm-AX620E-linux-uclibcgnueabihf-g++ \
    --disable-option-checking  --disable-debug --disable-optimize \
	--disable-fvisibility --disable-gch --disable-largefile \
  --disable-util --disable-dependency-tracking --disable-libtool-lock

make
echo "build finished"
