roslaunch stdr_launchers among_us.launch &
source spawn_robots.bash
python among_us/src/rviz_track_position.py &
python among_us/src/taskmaster.py &
roslaunch stdr_launchers among_rviz.launch
## roslaunch stdr_launchers robot_control.py
