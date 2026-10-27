#!/bin/bash

DIR=`pwd`
echo $DIR

./configure --prefix=$(DIR)/mp4v2-2.0.0/ --host=mips-linux  \
    CC=/opt/mips-gcc540-uclibc-64bit/bin/mips-linux-uclibc-gcc  CXX=/opt/mips-gcc540-uclibc-64bit/bin/mips-linux-uclibc-g++ \
	CFLAGS=-muclibc CPPFLAGS=-muclibc --disable-option-checking  --disable-debug --disable-optimize \
	--disable-fvisibility --disable-gch --disable-largefile \
  --disable-util --disable-dependency-tracking --disable-libtool-lock

make
echo "build finished"
