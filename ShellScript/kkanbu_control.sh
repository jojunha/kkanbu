#!/bin/sh

source ~/git/Kkanbu/devel/setup.bash

rosrun rosserial_python serial_node.py _port:=/dev/ttyACM2 _baud:=57600 