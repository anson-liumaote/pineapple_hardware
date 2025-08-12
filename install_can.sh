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
echo "*********************************"

# Add setserial low_latency to ~/.bashrc if not already present
BASHRC="$HOME/.bashrc"
LOW_LATENCY_SNIPPET='
# Automatically enable low_latency for all ttyUSB devices
for dev in /dev/ttyUSB*; do
    if [ -e "$dev" ]; then
        sudo setserial "$dev" low_latency
    fi
done
'

if ! grep -q "low_latency" "$BASHRC"; then
    echo "Adding low_latency script to $BASHRC..."
    echo "$LOW_LATENCY_SNIPPET" >> "$BASHRC"
else
    echo "low_latency script already present in $BASHRC, skipping."
fi

echo "*********************************"
echo "Done. Please restart your terminal or run:"
echo "source ~/.bashrc"
echo "*********************************"
