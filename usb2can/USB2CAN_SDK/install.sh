#! /bin/bash

arch=$(uname -m)
echo "*********************************"
echo "Install DingLAB USB2CAN SDK"
echo "device architecture: " $arch
echo "*********************************"

sudo cp include/*.h /usr/local/include/
sudo cp lib/$arch/*.so /usr/local/lib/
sudo cp share/usb_can.rules /etc/udev/rules.d/
cp lib/$arch/*.so python/pyusb2can

echo "*********************************"
echo "Install finished..."
echo "go to our github page: https://github.com/orgs/SOULDE-Studio"
echo "for motor control examples"
echo "*********************************"


