# trajectory_planning_gui
Qt based GUI for motion planning of a 6 DOF robotic arm.

### Installation
Clone the repository using:

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

### Requirements
* [6-dof-robotic-arm](https://github.com/anubhav1772/6-dof-robotic-arm) package
* [Eigen v3.4.0](http://www.eigen.tuxfamily.org/index.php?title=Main_Page#Download) - [Installation](https://github.com/anubhav1772/trajectory_planning_gui/blob/main/Eigen3_installation)
* ROS Noetic (Ubuntu 20.04)
* Gazebo v11.11.0
