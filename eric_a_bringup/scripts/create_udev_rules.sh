#!/bin/bash

echo ""
echo "This script copies ERIC_A udev rules to /etc/udev/rules.d/"
echo ""

echo ""
echo "OpenCR IMU (USB Serial) : /dev/ttyUSBx to /dev/imu :"
if [ -f "/etc/udev/rules.d/eric_a_imu.rules" ]; then
    echo "eric_a_imu.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4" ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyIMU" ' > /etc/udev/rules.d/99-opencr-cdc.rules

    echo 'eric_a_imu.rules created'
fi

echo ""
echo "YD LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyLiDAR :"
if [ -f "/etc/udev/rules.d/eric_a_lidar.rules" ]; then
    echo "eric_a_lidar.rules file already exist."
else 
    echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLIDAR"' >/etc/udev/rules.d/ydlidar.rules    

    echo 'eric_a_lidar.rules created'
fi


echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
