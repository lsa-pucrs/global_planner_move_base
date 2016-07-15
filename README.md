# global_planner_move_base

This is a simple package for running the ROS navigation stack's "move_base"
package along with the STAGE robot simulation package "stage_ros". It can be used as an example to implement different global planners.

## Getting Started

### Prerequisites
This package has been constructed using ROS Indigo in Linux Ubuntu 14.04 (Testing will be done using different ROS versions in the future).
Apart from ROS Indigo, the following packages are also required:

* stage_ros package (http://wiki.ros.org/stage_ros)
* move_base package (http://wiki.ros.org/move_base)
* amcl package (http://wiki.ros.org/amcl)
* map_server package (http://wiki.ros.org/map_server)
* rviz package (http://wiki.ros.org/rviz)

The user must also have a working catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

### Installation
* Download and extract (or simply clone) the master branch on your catkin workspace "src" folder.
* Install the package by running the "catkin_make" command on your catkin root folder.
* Set up the proper environmental variables via the "source <your catkin workspace>/devel/setup.bash" command.

## Usage
To use the package, simply run:
```
roslaunch path_planner planner.launch
```
in your terminal. If all the prerequisites are met, two new windows will appear, one with the STAGE environment and another one with RVIZ.

To move the robot, firstly specify an initial position in RVIZ with the "2D Pose Estimate" button on the top toolbar. Afterwards, specify a goal position with the "2D Nav Goal" button. The robot should start making its way towards the goal, following a green path in RVIZ's main view.

## Notes and Modifications
This package as-is uses ROS' native global planner (from the nav_core package) for path search. It is possible to modify the "planner.launch" file to use a user-defined planner. The "global_planner" file in the package's "scripts" folder is a python script which provides all the needed functionalities for ROS communication. The user must then only implement his/hers path search algorithm in the "search" function, which returns a message of the "Nav_msgs/Path" type.

If desired to change the map used in STAGE:
* Create a black and white image, where black corresponds to the obstacles in your environment.
* Change the "bitmap" field in the respective ".world" file to the name of your image.
* An OccupancyGrid map of this environment must be generated, which can be done with the "gmapping" package (http://wiki.ros.org/gmapping). First open the stage_ros package with your ".world" file, then run the command "rosrun gmapping slam_gmapping /scan:=/base_scan". You must drive the robot around the map many times to get a good map.
* Once mapping is complete, save the map with the map_server package. Run "rosrun map_server map_saver -f <filename>". Your map will be saved as a .pgm image and a YAML file.
* Modify the "planner.launch" file to use the new environment. 

## Troubleshooting
* RVIZ crashes unexpectedly before launching
Unknown bug, usually corrected by rerunning the "roslaunch path_planner planner.launch" command.
