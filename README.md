# ros_simple_pick_place
Simple pick &amp; place ros program with associated gui
# Simple Python Ros Pick and Place

This repository is a simple python pick and place application with a built in GUI and some additional features built into it.

<p align="center">
  <img src="./demo.gif" alt="gen3_lite rviz demo">
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

1. To start simulation run:
```sh
roslaunch pick_place_python spawn_gen3_lite.launch
```

2. To start pick and place control run:
```sh
roslaunch pick_place_python run_pick_place.launch
```
