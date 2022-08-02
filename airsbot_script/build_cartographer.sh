#!/bin/sh

cd ../airsbot_slam/cartographer/src/cartographer
mkdir -p build
sudo rm -r build/
mkdir -p build

cd build
cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX=/usr/local/stow/cartographer
ninja
CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install
cd /usr/local/stow
sudo stow cartographer

