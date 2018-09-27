# Minesweepers

## Quickstart

Add a symlink in your catkin workspace to this directory, then build the packages
```
catkin build seeker_map_builder seeker_msgs uavcan_bridge
```

In different windows run all nodes
```
rosrun seeker_map_builder map_builder
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun uavcan_bridge position_publisher.py /path/to/robot-software/uavcan_data_types/cvra/ slcan0
```

Then you can open the live map on your browser
```
firefox live_map.html
```

You can publish a fake mine position, it will be displayed in the live map (cross in the grid)
```
rostopic pub /mine_detection seeker_msgs/MineInfo "{ 'position': {'x': 14.5, 'y': 12.2} }" --once
```
