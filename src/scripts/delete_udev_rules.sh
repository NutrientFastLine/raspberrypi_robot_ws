#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  sllidar and base"
echo "sudo rm   /etc/udev/rules.d/raspberrypi_sllidar.rules"
sudo rm   /etc/udev/rules.d/raspberrypi_sllidar.rules

echo "sudo rm   /etc/udev/rules.d/raspberrypi_base.rules"
sudo rm   /etc/udev/rules.d/raspberrypi_base.rules

echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "finish  delete"
