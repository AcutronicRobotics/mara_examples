# MARA Examples

Please make sure you have followed the steps in [MARA repository](https://github.com/AcutronicRobotics/MARA#install) to set up ROS2, HRIM and dependencies.

## Build

Clone this repository in your ROS2 workspace.
```bash
cd ~/ros2_mara_ws/src && git clone https://github.com/AcutronicRobotics/mara_examples.git
```

Compile.
```bash
source /opt/ros/dashing/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install --packages-skip individual_trajectories_bridge
```

## Run

### Terminal 1

In order to run the examples you need to first spawn MARA in Gazebo.

```
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_gazebo mara.launch.py
```

You can choose between one of the available URDFs for spawning different configurations of MARA (`mara_robot_gripper_140`, `mara_robot_gripper_140_no_table`, `mara_robot_gripper_85` and `mara_robot_gripper_hande`). **Remember to spawn a URDF with gripper if you want to run the service examples**.

```
ros2 launch mara_gazebo mara.launch.py --urdf mara_robot_gripper_140
```

### Terminal 2

In a different terminal, run the example you would like to test.

```
source ~/ros2_mara_ws/install/setup.bash
ros2 run mara_minimal_publisher mara_minimal_publisher_v1
```
