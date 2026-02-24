# Localization Challenge

A 2D Monte Carlo Localization implementation.

## How to run
**First** on NoMachine Terminal: 
```
cd ~/mbot_ros_labs/src/mbot_localization/rviz
ros2 run rviz2 rviz2 -d localization.rviz
```
**Second** on VSCode Terminal #1:
```bash
ros2 run mbot_localization localization_node
```

**Last** on VSCode Terminal #2:
```bash
cd ~/mbot_ros_labs/src/mbot_rosbags/maze1
ros2 bag play maze1.mcap
```
## Algorithm Details

### Action Model
Particles are propagated using odometry with noise:
- Motion decomposed into: rotation₁ → translation → rotation₂
- Gaussian noise proportional to motion magnitude (alpha model)
- Parameters: k₁ (rotation noise), k₂ (translation noise)

### Sensor Model
Likelihood field matching:
- Each laser ray matched against occupancy grid distance field
- Hit probability: `exp(-distance² / 2σ²)`
- Mixture model: `0.85 × hit_prob + 0.15 × uniform`
- Bilinear interpolation for smooth distance queries

### Resampling
Low-variance systematic resampling:
- Eliminates low-weight particles (prevents particle deprivation)
- Duplicates high-weight particles
- Maintains filter diversity
