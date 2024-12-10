- [Assumptions](#assumptions)
- [Installation](#installation)
  * [In an already present workspace](#in-an-already-present-workspace)
  * [In a new workspace](#in-a-new-workspace)
- [Documentation](#documentation)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


# Assumptions
+ [Ros2 foxy](https://docs.ros.org/en/foxy/index.html) should be installed in the system
+ Using [ubuntu 20](https://releases.ubuntu.com/focal/)
# Installation
# Docker container requirements
If using this code inside of https://hub.docker.com/r/carms84/noetic_ros2 install these two packages: </br>
+ `sudo apt install ros-foxy-ros2-control`
+ `sudo apt install ros-foxy-ros2-controllers`\

There were some problems with the visualization of images in particular with image_tools. This seems to partially solve them:
+ `sudo apt-get install at-spi2-core`
## In an already present workspace
+ Clone this repo inside the src folder of a ros2 foxy workspace
+ build the workspace
+ source the project
## In a new workspace
+ `mkdir -p MyWorkspace/src`
+ `cd MyWorkspace/src`
+ `git clone https://github.com/Tonelllo/exprob1_DT.git`
+ `cd ..`
+ `source /opt/ros/foxy/setup.bash`
+ `colcon build --packages-select exprob_dt`
+ `source install/setup.bash`
# Execution
```bash
ros2 launch exprob_dt assignment1.launch.py
```
Note that a lxterminal window will be opened where you can select either robot or camera to be moved in order to detect the markers.
# Documentation
You can find the documentation at [https://tonelllo.github.io/exprob1_DT/html/](https://tonelllo.github.io/exprob1_DT/html/)
