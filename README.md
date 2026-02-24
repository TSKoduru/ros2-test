# mbot_ros_labs

This is the repository for classes use ROB2 MBot. Each folder in this repository is a ros package. And each one of them could be a standalone challenge.

To use:
1. Make a directory
    ```bash
    cd ~
    mkdir mbot_ros_labs
    ```
2. Clone this repository as `src`
    ```bash
    cd ~/mbot_ros_labs
    gh repo clone mbot-project/mbot_ros_labs src
    ```
3. Compile the packages
    ```bash
    colcon build
    source install/setup.bash
    ```