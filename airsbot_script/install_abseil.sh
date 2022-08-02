#!/bin/sh

set -o errexit
set -o verbose

# Move to lib directory.
cd ../airsbot_lib

# Clean folder.
mkdir -p abseil-cpp
sudo rm -r abseil-cpp

# Unzip abseil-cpp.tar.gz
tar -zxvf abseil-cpp.tar.gz

# Build.
cd abseil-cpp
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl \
  ..
ninja
sudo ninja install
cd /usr/local/stow
sudo stow absl

# Clean build folder.
cd -
cd ../../
sudo rm -r abseil-cpp
