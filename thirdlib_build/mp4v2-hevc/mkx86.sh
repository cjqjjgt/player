#!/bin/bash

#cd mp4v2-2.0.0
DIR=`pwd`
echo $DIR
./configure --prefix=/home/work/temp/mp4v2x86

make
make install
echo "build finished"
