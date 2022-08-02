#!/bin/sh

set -o errexit
set -o verbose

# Move to lib directory.
cd ../airsbot_lib

# Clean folder.
mkdir -p protobuf-3.4.1
sudo rm -r protobuf-3.4.1

# Unzip ceres-solver.tar.gz
tar -zxvf protobuf.tar.gz

# Build.
cd protobuf-3.4.1
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  ../cmake
ninja
sudo ninja install

# Clean build folder.
cd -
cd ..
sudo rm -r protobuf-3.4.1
