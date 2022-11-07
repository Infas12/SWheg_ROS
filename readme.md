# SWheg : Wheel-leg transformable robot

## Introduction 
TBD


## Requirements

- Ubuntu 18.04
- ROS melodic
- SMach
- Gazebo9
- Joy

## Quick Start

First, clone and compile the code in an catkin workspace.

To start the Simulation, use:
```
roslaunch wheelleg_gazebo WheelLegHexapod.launch
```
or
```
roslaunch wheelleg_gazebo WheelLegQuadraped.launch 
```

Then, in the second terminal, start the `joy` node.
```
rosrun joy joy_node 
```

Finally, launch the corresponding controller:

```
rosrun wheelleg_hexapod_control SmachTest.py
```

or

```
rosrun wheelleg_quadruped_control SmachTest.py
```

