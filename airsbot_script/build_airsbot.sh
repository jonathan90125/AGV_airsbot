#!/bin/sh

cd ../../..
sudo rm -r build/ devel/
catkin_make -DCATKIN_BLACKLIST_PACKAGES="cartographer"
