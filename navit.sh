#!/bin/sh

sudo systemctl start gpsd.socket

navit -c /home/pi/.navit/navit_reform.xml