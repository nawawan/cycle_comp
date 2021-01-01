#!/bin/sh

sudo systemctl stop gpsd.socket

python3 /home/pi/cycle_comp/mobcyccom.py