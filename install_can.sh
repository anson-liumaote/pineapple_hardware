#! /bin/bash

echo "*********************************"
echo "Apply DingLAB USB2CAN udev rules"
echo "*********************************"

# Write udev rules for USB2CAN devices
cat <<EOF | sudo tee /etc/udev/rules.d/usb_can.rules > /dev/null
KERNEL=="ttyACM*", ATTRS{idVendor}=="1483", ATTRS{idProduct}=="1110",MODE:="0666", SYMLINK+="USB2CAN0"
KERNEL=="ttyACM*", ATTRS{idVendor}=="1483", ATTRS{idProduct}=="1111",MODE:="0666", SYMLINK+="USB2CAN1"
KERNEL=="ttyACM*", ATTRS{idVendor}=="1483", ATTRS{idProduct}=="1112",MODE:="0666", SYMLINK+="USB2CAN2"
KERNEL=="ttyACM*", ATTRS{idVendor}=="1483", ATTRS{idProduct}=="1113",MODE:="0666", SYMLINK+="USB2CAN3"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "*********************************"
echo "Udev rules applied."