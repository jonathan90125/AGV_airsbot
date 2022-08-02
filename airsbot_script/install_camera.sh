#!/bin/sh

sudo apt-get install build-essential freeglut3 freeglut3-dev ros-melodic-libuvc-ros libsfml-dev

cd ../airsbot_lib
tar -zxvf AstraSDK.tar.gz
mv AstraSDK-v2.1.3-94bca0f52e-20210608T062039Z-Ubuntu18.04-x86_64 AstraSDK
cd AstraSDK
cd install
sudo chmod 777 ./install.sh
sudo ./install.sh
cd ..
cd bin
./SimpleColorViewer-SFML

 
