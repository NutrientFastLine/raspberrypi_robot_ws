#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  sllidar and base"
echo "rplidar usb connection as /dev/raspberrypi_sllidar , check it using the command : ls -l /dev|grep ttyUSB"
echo "rplidar usb connection as /dev/raspberrypi_base , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy raspberrypi_sllidar.rules to  /etc/udev/rules.d/"
sudo cp raspberrypi_sllidar.rules  /etc/udev/rules.d
echo "start copy raspberrypi_base.rules to  /etc/udev/rules.d/"
sudo cp raspberrypi_base.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "finish "
