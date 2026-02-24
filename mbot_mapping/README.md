# Mapping challenge

ROS 2 package for odometry-based occupancy grid mapping.

## Package Contents

### Executables
- `mapping_node.cpp`: Main mapping node that subscribes to lidar scans and odometry tf to build an occupancy grid map. Publishes the generated map to `/map` topic.

### Launch Files
- `mapping.launch.py`: Launches the mapping node with all necessary components.
- `view_map.launch.py`: Launches a node to publish a saved map to `/map` topic and `map` frame.


## Run

**Terminal 1**: 
```bash
ros2 launch mbot_bringup mbot_bringup.launch.py
```
- Brings up the lidar driver, robot description for visualization, and static transforms.

**Terminal 2** - Start the mapping node:
```bash
ros2 launch mbot_mapping mapping.launch.py
```
- This will create and publish the occupancy grid map to `/map` topic.

**Terminal 3** - After mapping the entire area, save the map:
```bash
cd ~/mbot_ros_labs/src/mbot_mapping/maps
ros2 run nav2_map_server map_saver_cli -f map_name
```

**To view a saved map**
```bash
ros2 launch mbot_mapping view_map.launch.py map_name:=your_map
```
- This publishes the map to `/map` topic and `map` frame for visualization.

Or you can simply install a VSCode Extension to view pgm file.