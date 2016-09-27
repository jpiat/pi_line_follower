#!/bin/bash

echo "Compiling latest version of code in RUN mode"
touch tests/polypheme.cpp
make polypheme MODE=-DRUN
sync
read -p "Make sure emergency stop is dis-engaged and press key" -n1 -s
echo "Press switch to initiate start and release to start 2s countdown"
echo "Disconnect usb serial interface"
sudo ./polypheme
sync
echo "Engage emergency stop"
