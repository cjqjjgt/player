#!/bin/bash

cd mp4v2-2.0.0
DIR=`pwd`
echo $DIR

./configure --prefix=$(DIR)/mp4v2-2.0.0/ --host=arm-linux  \
    CC=arm-fullhanv3-linux-uclibcgnueabi-gcc  CXX=arm-fullhanv3-linux-uclibcgnueabi-g++ \
	--disable-option-checking  --disable-debug --disable-optimize \
	--disable-fvisibility --disable-gch --disable-largefile \
  --disable-util --disable-dependency-tracking --disable-libtool-lock

make
echo "build finished"
