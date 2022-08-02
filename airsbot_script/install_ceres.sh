#!/bin/sh

set -o errexit
set -o verbose

# The version of ceres-solver is 1.13.0
VERSION="1.13.0"

# Move to lib directory.
cd ../airsbot_lib

# Clean folder.
mkdir -p ceres-solver
sudo rm -r ceres-solver

# Unzip ceres-solver.tar.gz
tar -zxvf ceres-solver.tar.gz

# Build.
cd ceres-solver
mkdir build
cd build
cmake .. -G Ninja -DCXX11=ON -DCMAKE_INSTALL_PREFIX=/usr/local/stow/ceres-solver
ninja
CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install
cd /usr/local/stow
sudo stow ceres-solver

# Clean build folder.
cd -
cd ../../
sudo rm -r ceres-solver
