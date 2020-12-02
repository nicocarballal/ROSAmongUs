source ./../devel/setup.bash
roslaunch stdr_launchers among_us.launch &
source spawn_robots.bash
python among_us/src/rviz_track_position.py &
python among_us/src/taskmaster.py &
roslaunch stdr_launchers among_rviz.launch &
roslaunch among_us occupancy_grid.launch &
python among_us/src/robot_control.py 
