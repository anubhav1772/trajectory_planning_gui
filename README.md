# trajectory_planning_gui
A C++ Qt-based software controller for a 6 DOF robotic arm, with ROS backend.

### Installation
Clone the repositories using:

    git clone https://github.com/anubhav1772/6-dof-robotic-arm.git
    git clone https://github.com/anubhav1772/trajectory_planning_gui.git

Run catkin_make in your ROS source directory

    $ cd ~/catkin_ws
    $ catkin_make

Start the simulation using:

    $ roslaunch owr_gazebo owr_spawn.launch

Launch moveit and rviz:

    $ roslaunch owr_moveit_config owr_simulation_execution.launch
    
Launch GUI:

    $ roslaunch trajectory_planning_gui trajectory_planning_gui.launch
    
(Qt Specific):
    
    $ sudo apt install qtcreator
    $ sudo apt install libqt5multimedia5-plugins qtmultimedia5-dev

### Cartestian-based trajectory planning (straight line path) 
<img src="https://drive.google.com/uc?export=view&id=1UoxxS-7pokdOwdiFl92AJ5L-Jqb-CuxH" height="80%" width="80%"/>

### Requirements
* [6-dof-robotic-arm](https://github.com/anubhav1772/6-dof-robotic-arm) package
* [Eigen v3.4.0](http://www.eigen.tuxfamily.org/index.php?title=Main_Page#Download) - [Installation](https://github.com/anubhav1772/trajectory_planning_gui/blob/main/Eigen3_installation)
* Qt5Widgets, Qt5Multimedia and Qt5MultimediaWidgets
* ROS Noetic (Ubuntu 20.04)
* Gazebo v11.11.0
