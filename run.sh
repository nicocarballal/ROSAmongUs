source ./../devel/setup.bash
roslaunch stdr_launchers among_us.launch x:=13 y:=10 &
source spawn_robots.bash 
python among_us/src/rviz_track_position.py &
python among_us/src/task_creator.py &
#python among_us/src/a_star_function.py &
python among_us/src/taskmaster.py robot0 &
python among_us/src/taskmaster.py robot1 &
python among_us/src/taskmaster.py robot2 &
python among_us/src/taskmaster.py robot3 &
python among_us/src/taskmaster.py robot4 &
python among_us/src/taskmaster.py robot5 &
python among_us/src/taskmaster.py robot6 &
python among_us/src/taskmaster.py robot7 &
roslaunch stdr_launchers among_rviz.launch &
roslaunch among_us occupancy_grid_combined.launch &
python among_us/src/robot_control.py robot0 &
python among_us/src/robot_control.py robot1 &
python among_us/src/robot_control.py robot2 & 
python among_us/src/robot_control.py robot3 &
python among_us/src/robot_control.py robot4 &
python among_us/src/robot_control.py robot5 &
python among_us/src/robot_control.py robot6 &
python among_us/src/robot_control.py robot7 &

#python among_us/src/robot_control1.py &
