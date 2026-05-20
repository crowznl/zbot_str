#!/bin/bash
rm -rf build
mkdir build
cd build
cmake -DUSE_ZBOT_RL_ONE=ON ..
make -j
