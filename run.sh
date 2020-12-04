source ./../devel/setup.bash
roslaunch stdr_launchers among_us.launch &
source spawn_robots.bash
python among_us/src/rviz_track_position.py &
python among_us/src/task_creator.py &
python among_us/src/taskmaster.py robot0 &
python among_us/src/taskmaster.py robot2 &
roslaunch stdr_launchers among_rviz.launch &
roslaunch among_us occupancy_grid.launch &
python among_us/src/robot_control.py robot0 &
python among_us/src/robot_control.py robot2
#python among_us/src/robot_control1.py &
