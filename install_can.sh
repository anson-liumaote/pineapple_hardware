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

# ---- FTDI FT4232H (VID:0403, PID:6011) 固定從 rs485-1 開始 ----
# 用 USB 介面號 bInterfaceNumber 來對應四個通道
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", ENV{ID_USB_INTERFACE_NUM}=="00", MODE:="0666", GROUP:="dialout", SYMLINK+="rs485-1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", ENV{ID_USB_INTERFACE_NUM}=="01", MODE:="0666", GROUP:="dialout", SYMLINK+="rs485-2"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", ENV{ID_USB_INTERFACE_NUM}=="02", MODE:="0666", GROUP:="dialout", SYMLINK+="rs485-3"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", ENV{ID_USB_INTERFACE_NUM}=="03", MODE:="0666", GROUP:="dialout", SYMLINK+="rs485-4"

# ---- FTDI FT232H (Single) 命名為 rs485-a1 ----
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0666", GROUP:="dialout", SYMLINK+="rs485-a1"

# --- Best-effort: enable low_latency via setserial on any ttyUSB*/ttyACM*
# (Some drivers ignore setserial; rule is still safe.)
ACTION=="add", KERNEL=="ttyUSB*", RUN+="/usr/bin/setserial /dev/%k low_latency"
ACTION=="add", KERNEL=="ttyACM*", RUN+="/usr/bin/setserial /dev/%k low_latency"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "*********************************"
echo "Udev rules applied."
echo "*********************************"
