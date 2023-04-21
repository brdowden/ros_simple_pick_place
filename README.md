# ros_simple_pick_place
Simple pick &amp; place ros program with associated gui
# Simple Python Ros Pick and Place

This repository is a simple python pick and place application with a built in GUI and some additional features built into it.

<p align="center">
  <img src="./img.png" alt="sample space">
</p>

## Structure of package
```sh
.
├── CMakeLists.txt
├── package.xml
├── src
└── scripts
    └── my_gui.py
    └── pick_place.py
    └── publisher_box.py
    └── random_joints.py
    
```

## Installation
 - Package depends on [moveit](https://github.com/ros-planning/panda_moveit_config.git) and was tested on `noetic-devel` version of it. 

 - Thus install `panda_moveit_config` by following instructions in the repository.

 - Add this package to your `src` folder with:

```sh
git clone https://github.com/brdowden/ros_simple_pick_place.git
```

 - And build the workspace with:

```sh
catkin build
```

## Usage
1. in your built workspace, edit the directory of your files in the 'my_gui.py' file at lines 77, 82, & 87 to point to your installed package. For me they were at '/home/ros/ws_moveit/src/simple_pick_place/scripts/pick_place.py'.

2. To start simulation run:
```sh
roslaunch panda_moveit_config demo.launch
```
in the rviz environment add the following variables:
- rviz/RobotModel
- moveit_ros_visualization/RobotState
- moveit_ros_visualization/PlanningScene

And uncheck MotionPlanning

3. To start pick and place GUI run:
```sh
rosrun simple_pick_place my_gui.py 
```

The gui will have 3 buttons:
1. 'set joints to random position'
2. 'run pick_place'
3. 'enable box pose updates'

On start click 'enable box pose updates' which will start the box state feedback in th gui

You can then run 'set joints to random position' to have the robot move to a random position. Please note that singularities are possible with the script will require the pick place module to be ran or the environment to be restarted.

Lastly, you can click 'run pick_place' to execute the pick place action. You can restart the process by reclicking the button.
