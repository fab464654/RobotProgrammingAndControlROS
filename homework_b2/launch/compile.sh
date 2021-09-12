#!/bin/sh

gnome-terminal -- /bin/sh -c 'sleep 0.5; echo Compiling the Catkin environment...; cd catkin_ws/; sleep 0.5; catkin_make clean; sleep 0.5; catkin_make; cd ..; exec bash'


