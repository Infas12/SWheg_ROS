# SWheg : Wheel-leg transformable robot

## Overview
This repository holds the code for SWheg, a tendon-driven wheel-leg transformable robot.




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

On the real robot, use:
```
rosrun wheelleg_real motorController
```

Then, in the second terminal, start the `joy` node.
```
rosrun joy joy_node 
```

Finally, launch the corresponding controller in another terminal:

```
rosrun wheelleg_hexapod_control SmachTest.py
```

or

```
rosrun wheelleg_quadruped_control SmachTest.py
```

