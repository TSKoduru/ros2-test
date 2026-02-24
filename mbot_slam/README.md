# SLAM Challenge

Particle filter based SLAM.

## How to run
**First** on NoMachine Terminal: 
```
cd ~/mbot_ros_labs/src/mbot_slam/rviz
ros2 run rviz2 rviz2 -d slam.rviz
```
**Second** on VSCode Terminal #1:
```bash
ros2 run mbot_slam slam_node
```

**Last** on VSCode Terminal #2:
```bash
cd ~/mbot_ros_labs/src/mbot_rosbags
ros2 bag play slam_test
```

**Save map** on VSCode Terminal #3:
```bash
cd ~/mbot_ros_labs/src/mbot_slam/maps
ros2 run nav2_map_server map_saver_cli -f map_name
```
