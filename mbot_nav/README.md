# Navigation Challenge

## Path Planning
### Unit Test
```bash
ros2 run mbot_nav astar_test
```
### Testing mode
1. Run launch file to publish map and run nagivation node
    ```bash
    ros2 launch mbot_nav path_planning.launch.py map_name:=maze1
    ```
2. Open Rviz to set initial pose and goal pose
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```

Just run the node: 
```bash
ros2 run mbot_nav navigation_node
```
### Real-world mode (with localization)
1. Launch the navigation node with the map publish
    ```bash
    ros2 launch mbot_nav path_planning.launch.py map_name:=your_map pose_source:=tf
    ```
2. Run your localization node
3. Start rviz and set initial pose, localization node needs it to initialize particles
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```
4. Then set the goal pose, the navigation node needs it


Just run the node: 
```bash
ros2 run mbot_nav navigation_node --ros-args -p pose_source:=tf
```

## Exploration
1. Start rviz for visualization
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```
2. Run slam node
3. Then exploration node
    ```bash
    ros2 run mbot_nav exploration_node
    ```