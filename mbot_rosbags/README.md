# MBot ROS Bags

```bash
ros2 bag play maze1.mcap
```
## Collect ros bag for localization
First map the maze, then run nav2_acml to have "ground truth" pose, then run the following:
```bash
ros2 bag record \
/scan \
/odom \
/tf \
/tf_static \
/amcl_pose \
/map \
/initialpose \
/cmd_vel \
/imu
```

## Collect ros bag for SLAM
```bash
ros2 bag record \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /imu \
  /cmd_vel \
  -o slam_test_bag
```