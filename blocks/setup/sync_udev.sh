#!/bin/bash
set -e

echo "Copying udev rules..."
echo "   99-blocks.rules"
sudo cp $PWD/99-blocks.rules /etc/udev/rules.d/

echo "Copying latency fix..."
sudo mkdir -p /etc/udev/scripts
echo "   set_ftdi_low_latency.sh"
sudo cp $PWD/set_ftdi_low_latency.sh /etc/udev/scripts/

echo "Reloading UDev rules..."
sudo udevadm control --reload-rules

echo "DONE!"
echo "Please unplug and replug all your devices for changes to take effect."

