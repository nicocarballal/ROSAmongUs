# AmongUsROS

## Current abilities

What you need to do to run:

```bash
roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch

```

```bash
roslaunch stdr_launcher rviz.launch
```
Ensure that the rostopic "stdr_server/sources_visualization_marker" is being tracked in RVIZ by SourceMarkers


What each of the scripts in src/among_us/src do:

### test_publish_circle.py 
Just publishes a marker into RVIZ 
### test_publish_circle_at_robot_position.py 
publishes a marker at the position when it subscribes to 'robot0/odom' 
### test_color_variation 
tests color variation in marker appearance




