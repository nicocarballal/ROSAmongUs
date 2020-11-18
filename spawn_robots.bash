#!/bin/bash

echo -e "Load in the robots: "


##Commented lines do not work with spawning robots for some reason so just run them separately in terminals for now 

#roslaunch stdr_launchers among_us.launch x:=10 y:=10

rosrun stdr_robot robot_handler add ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/resources/robots/among_us1.yaml 12 10 0

rosrun stdr_robot robot_handler add ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/resources/robots/among_us1.yaml 11 9 3.14

rosrun stdr_robot robot_handler add ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/resources/robots/among_us1.yaml 12 8 1.57

## Can uncomment below to track robot position with markers automatically in RVIZ
#python ~/ros_workspaces/project/src/among_us/src/rviz_track_position.py

#roslaunch stdr_launchers rviz.launch



