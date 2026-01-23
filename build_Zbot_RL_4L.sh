#!/bin/bash
rm -rf build
mkdir build
cd build
cmake -DUSE_ZBOT_RL_4L=ON ..
make -j
