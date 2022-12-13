#!/bin/bash

echo ""
echo "This script copies ERIC_A udev rules to /etc/udev/rules.d/"
echo ""

echo ""
echo "ERIC_A IMU (USB Serial) : /dev/ttyUSBx to /dev/ttyIMU :"
if [ -f "/etc/udev/rules.d/eric_a_imu.rules" ]; then
    echo "eric_a_imu.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty*", ATTRS{idVendor}=="10c4" ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyIMU" ' > /etc/udev/rules.d/eric_a_imu.rules

    echo 'eric_a_imu.rules created'
fi

echo ""
echo "ERIC_A LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyLiDAR :"
if [ -f "/etc/udev/rules.d/eric_a_lidar.rules" ]; then
    echo "eric_a_lidar.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="1111", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLIDAR"' >/etc/udev/rules.d/eric_a_lidar.rules    

    echo 'eric_a_lidar.rules created'
fi

echo ""
echo "ERIC_A dynamixel (USB Serial) : /dev/ttyUSBx to /dev/ttyDYNAMIXEL :"
if [ -f "/etc/udev/rules.d/eric_a_dynamixel.rules" ]; then
    echo "eric_a_dynamixel.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT78LEKI", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyDYNAMIXEL"' >/etc/udev/rules.d/eric_a_dynamixel.rules    

    echo 'eric_a_dynamixel.rules created'
fi

echo ""
echo "ERIC_A arduino (USB Serial) : /dev/ttyACMx to /dev/ttyARDUINO :"
if [ -f "/etc/udev/rules.d/eric_a_arduino.rules" ]; then
    echo "eric_a_arduino.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", ATTRS{serial}=="9593132303235180C0A1", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyARDUINO"' >/etc/udev/rules.d/eric_a_arduino.rules    

    echo 'eric_a_arduino.rules created'
fi


echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
