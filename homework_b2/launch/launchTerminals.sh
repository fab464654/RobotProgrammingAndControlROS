#!/bin/sh

gnome-terminal -- /bin/sh -c 'sleep 0.5; echo Launching the Gazebo environment for ur5 robot...; roslaunch ur_gazebo ur5.launch limited:=true; exec bash'

gnome-terminal -- /bin/sh -c 'sleep 6; echo Launching the MoveIt environment for ur5 robot...; roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true; exec bash'

gnome-terminal -- /bin/sh -c  'sleep 12; echo Launching the RViz environment for ur5 robot...; roslaunch ur5_moveit_config moveit_rviz.launch config:=true; exec bash'

gnome-terminal -- /bin/sh -c  'sleep 16; echo Launching MoveIt! simulation environment for ur5 robot...; rosrun homework_b2 move_group_interface_ur5; exec bash'


