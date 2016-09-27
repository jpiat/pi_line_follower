#!/bin/bash

read -p "Make sure emergency stop is dis-engaged and press key" -n1 -s
echo "Press switch to initiate start and release to start countdown"
echo "Disconnect usb serial interface"
sudo ./polypheme
echo "Engage emergency stop"
