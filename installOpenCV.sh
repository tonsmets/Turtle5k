#!/bin/bash
echo "## OpenCV Dependencies installer ##"
if [[ $EUID -ne 0 ]]; then
   echo "ERROR: This script must be run as root!"
   exit 1
fi
apt-get install libopencv-dev build-essential checkinstall cmake pkg-config yasm libtiff4-dev libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev
