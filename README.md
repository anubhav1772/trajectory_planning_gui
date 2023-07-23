# trajectory_planning_gui
Qt based GUI for motion planning of a 6 DOF robotic arm.

### Installation
---

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


ROS - Noetic

Gazebo version 11.11.0
