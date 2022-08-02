#!/bin/sh

# Move to lib directory.
cd ../airsbot_lib

# Clean folder.
mkdir -p libmodbus-3.1.6
sudo rm -r libmodbus-3.1.6

# Unzip libmodbus-3.1.6.tar.gz
tar -zxvf libmodbus-3.1.6.tar.gz

# Build.
cd libmodbus-3.1.6
sudo ./configure
sudo make
sudo make install

# Clean build folder.
cd ..
sudo rm -r libmodbus-3.1.6
