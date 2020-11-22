# AmongUsROS

## Current abilities

What you need to do to run:

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
Ensure that the rostopic "stdr_server/sources_visualization_marker" is being tracked in RVIZ by SourceMarkers

## Backend
Within src/stdr_simulator, I've made a few changes to files for launching 

| File            | Path                                                                        | What it does                                                                                                                                       |
|-----------------|-----------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| among_us.launch | ~/ros_workspaces/project/src/stdr_simulator/stdr_launchers/launch           | Opens the map yaml and specifies robot parameters according to among_us1.yaml file in the resources/robots directory                               |
| among_us.yaml   | ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/maps/among_us    | Specifies the map for among us. Used adobe illustrator to get the walls of the actual among us map which is stored in among-us-edges-fixed-ai.png. |
| among_us1.yaml  | ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/resources/robots | Specifies robot parameters. For right now, it's a slight variation  of pandora_robot.yaml in the same folder.                                      |
| among_us.xml    | ~/ros_workspaces/project/src/stdr_simulator/stdr_resources/resources/robots | Calls the yaml file. Direct copy except for one change of pandora_robot.xml                                                                        |
| among_rviz.launch    | ~/ros_workspaces/project/src/stdr_simulator/stdr_launchers/launch | Hosts the parameters for rviz to open manually tracking everything we want it to trackk so far                                                                        |

### rviz_track_position.py
Currently, this tracks the position of robots 0 through 3 and sets markers in RVIZ

What each of the scripts in src/among_us/src do (Not too important):
### test_publish_circle.py 
Just publishes a marker into RVIZ 
>>>>>>> 81f46e8617d9f179984d7775d06e93906777b6d6
### test_color_variation 
tests color variation in marker appearance




