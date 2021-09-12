#!/bin/sh

gnome-terminal -- /bin/sh -c 'sleep 0.5; echo Launching the RViz environment for the designed SCARA robot...; roslaunch homework_b1 display.launch model:='catkin_ws/src/homework_b1/urdf/scara.urdf'; exec bash'

gnome-terminal -- /bin/sh -c 'sleep 5; echo Launching the inverse.py script to compute inverse kinematics...; rosrun homework_b1 inverse.py; exec bash'

gnome-terminal -- /bin/sh -c  'sleep 6; echo To call the inverseKinematics service you should provide target coordinates in the following manner:; echo Example: rosservice call /inverseKinematics -- 0.2 0.45 0.36 1; exec bash'

gnome-terminal -- /bin/sh -c  'sleep 10; echo Show some information about active TOPICS:; rostopic list; echo; echo Show some information about active SERVICES:; rosservice list; exec bash'

