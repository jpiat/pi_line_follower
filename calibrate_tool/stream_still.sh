#!/bin/bash
gpio export 17 in
gpio mode 17 up
gpio mode 0 up
echo "Copy the following to your host computer console :"
echo "nc -l -p 5000 | mplayer -fps 60 -cache 1024 -"
read -p "Press key when done... " -n1 -s
./capture_calibrate ~/Pictures | nc 192.168.7.1 5000

