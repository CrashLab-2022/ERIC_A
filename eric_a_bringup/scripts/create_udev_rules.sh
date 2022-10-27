#!/bin/bash

echo ""
echo "This script copies ERIC_A udev rules to /etc/udev/rules.d/"
echo ""

# echo ""
# echo "OpenCR IMU (USB Serial) : /dev/ttyACMx to /dev/imu :"
# if [ -f "/etc/udev/rules.d/99-opencr-cdc.rules" ]; then
#     echo "99-opencr-cdc.rules file already exist."
# else 
#     echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483" ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyIMU" ' > /etc/udev/rules.d/99-opencr-cdc.rules

#     echo '99-opencr-cdc.rules created'
# fi

echo ""
echo "YD LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyLiDAR :"
if [ -f "/etc/udev/rules.d/ydlidar.rules" ]; then
    echo "ydlidar.rules file already exist."
else 
    echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLIDAR"' >/etc/udev/rules.d/ydlidar.rules    

    echo 'ydlidar.rules created'
fi


echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
