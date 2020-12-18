# AmongUsROS

## Current abilities
To run everything you need to start up our project run the bash file:
```bash
source run.sh
```

These are the commands the bash file runs:

```bash
roslaunch stdr_launchers among_us.launch

```
Spawn in the robots from root directory
```bash
source spawn_robots.bash
```
Run the tracking python script (src/among_us/src) to give visual in RVIZ of location of robots
```bash
python rviz_track_position.py
```
Run the taskmaster script (src/among_us/src) to start publishing tasks
```bash
python taskmaster.py
```
Launch RVIZ
```bash
roslaunch stdr_launchers among_rviz.launch
```
Run the controller script (src/among_us/src) to have your robot0 go to task1
```bash
roslaunch stdr_launchers robot_control.py
```
Ensure that the rostopic "stdr_server/sources_visualization_marker" is being tracked in RVIZ by SourceMarkers

## Backend
Within src/stdr_simulator, I've made a few changes to files for launching 

| File            | Path                                                                        | What it does                                                                                                                                       |
|-----------------|-----------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| among_us.launch | ~/ros_workspaces/project/src/stdr_simulator/stdr_launchers/launch           | Opens the map yaml and specifies robot parameters according to among_us1.yaml file in the resources/robots directory                               |
| among_us.yaml   | ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/maps/among_us    | Specifies the map for among us. Used adobe illustrator to get the walls of the actual among us map which is stored in among-us-edges-fixed-ai.png. |
| among_us1.yaml  | ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/resources/robots | Specifies robot parameters. For right now, it's a slight variation  of pandora_robot.yaml in the same folder.                                      |
| among_us.xml    | ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/resources/robots | Calls the yaml file. Direct copy except for one change of pandora_robot.xml                                                                        |
| among_rviz.launch    | ~/ros_workspaces/project/src/stdr_simulator/stdr_launchers/launch | Hosts the parameters for rviz to open manually tracking everything we want it to track so far                                                                        |



## Custom Python Scripts

| File            | Path                                                                        | What it does                                                                                                                                       |
|-----------------|-----------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| parameter_server.py | ~/ros_workspaces/project/src/among_us/src           | Creates a subscriber object to receive Odometry message from the robots to update robot position parameters.                               |
| robot_controller.py | ~/ros_workspaces/project/src/among_us/src            | Controls a robot from their current position to go to a target position as dictated by the taskmaster. |
| taskmaster.py | ~/ros_workspaces/project/src/among_us/src             |  Consistently assigns crewmates to their tasks and assigns impostors a crewmate to kill. This is where the A* algorithm is run, and individual waypoints are sent to each of the crewmates, which simplifies controller implementation|
| kill_checker.py | ~/ros_workspaces/project/src/among_us/src           | Continuously checks if an impostor is in close enough range to kill a crewmate. If so, the impostor kills the crewmate and the proper parameters are updated. |
| game_ending_checker.py | ~/ros_workspaces/project/src/among_us/src           | Continuously checks if crewmates completed all tasks or if impostors have killed all the crewmates. If crewmates complete all tasks before the impostors have a chance to kill all crewmates, then the crewmates win the game. Otherwise, the impostors win. |
| occupancy_grid_2d.py | ~/ros_workspaces/project/src/among_us/src           | Initialized by running mapping_node.py and controlling one robot using STDR Tele-op Keyboard and moving the robot through the map. As the game progresses, all robots will contribute to the occupancy grid as they move through the map. Path planning is performed on the continuously updating occupancy grid. |
| path_publisher.py | ~/ros_workspaces/project/src/among_us/src            | Continuously checks for the robot path parameter and publishes it to RVIZ for visualization purposes. |
                





