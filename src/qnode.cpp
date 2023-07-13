/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/trajectory_planning_gui/qnode.hpp"
#include <std_msgs/UInt8.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace trajectory_planning_gui {

  /*****************************************************************************
  ** Implementation
  *****************************************************************************/

  QNode::QNode(int argc, char** argv) 
      : init_argc(argc),
        init_argv(argv)
      {}

  QNode::~QNode()
  {
    if(ros::isStarted()) 
    {
      ArmClient.reset();
      arm_move_group.reset();
      kinematic_state.reset();

      manager_->stopUpdate();

      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init(QLabel* qLabel_1)
  {
    ros::init(init_argc,init_argv,"trajectory_planning_gui");
    if (!ros::master::check()) 
    {
      return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh;

    // rosparam list|grep robot_description_semantic
    // rosparam list|grep robot_description
    if(!nh.hasParam("/robot_description") || !nh.hasParam("/robot_description_semantic"))
    {
      return false;
    }

    // TEACH MODE
    teach_pub = nh.advertise<std_msgs::UInt8>("teach_signal", 1);
    teach_mode_flag = false;

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

    imageViewLabel_1 = qLabel_1;
    
    /*! Initialize the MoveIt parameters:
        - MoveIt group
        - Kinematic State is the current kinematic configuration of the Robot
        - Robot model which handles getting the Robot Model
        - Joint Model group which are necessary for checking if Way-Point is outside the IK Solution
    */

    // Moveit 
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    // move group arm
    // arm_move_group = new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP);
    arm_move_group = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP));
    arm_move_group->setPoseReferenceFrame(frame_id_);

    // choose the RRTConnect planner
    // arm_move_group->setPlannerId("RRTConnect");

    kinematic_state = moveit::core::RobotStatePtr(arm_move_group->getCurrentState());
    kinematic_state->setToDefaultValues();

    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(frame_id_));

    // arm_move_group->setStartStateToCurrentState();
    
    // Scaling factors for optionally reducing the maximum joint velocities and
    // accelerations.  Allowed values are in (0,1].  The maximum joint velocity and
    // acceleration specified in the robot model are multiplied by thier respective
    // factors.  If either are outside their valid ranges (importantly, this
    // includes being set to 0.0), the factor is set to the default value of 1.0
    // internally (i.e., maximum joint velocity or maximum joint acceleration).
    
    // arm_move_group->setMaxVelocityScalingFactor(1.0);
    // arm_move_group->setMaxAccelerationScalingFactor(1.0);

    // arm_joint_model_group = arm_move_group->getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
    // ROS_INFO_STREAM(*arm_move_group->getCurrentState());

    const robot_model::RobotModelConstPtr &kmodel = kinematic_state->getRobotModel();
    arm_joint_model_group = kmodel->getJointModelGroup(ARM_PLANNING_GROUP);

    ROS_INFO_NAMED("", "End effector link: %s", arm_move_group->getEndEffectorLink().c_str());

    max_velocity_scaling_factor = 1;
    get_joint_max_velocities();
    setCartParams(10.0, 0.05, 0.0, true, true);

    move_joint_pub_ = nh.advertise<trajectory_planning_gui::MoveJoint>("move_joint", 1);
    init_joint_control();

    jog_frame_pub_ = nh.advertise<trajectory_planning_gui::JogFrame>("jog_frame", 1);
    // Get groups parameter of jog_frame_node
    nh.getParam("/jog_frame_node/group_names", group_names_);
    for (int i=0; i<group_names_.size(); i++)
    {
      ROS_INFO_STREAM("group_names:" << group_names_[i]);
    }
    nh.getParam("/jog_frame_node/link_names", link_names_);
    for (int i=0; i<link_names_.size(); i++)
    {
      ROS_INFO_STREAM("link_names:" << link_names_[i]);
    }

    jog_joint_pub_ = nh.advertise<trajectory_planning_gui::JogJoint>( "jog_joint", 1);
    nh.getParam("/jog_joint_node/joint_names", joint_name_);
    jog_joint_num_ = joint_name_.size();
    jog_value_.resize(jog_joint_num_);

    init_jog_mode();
    init_robot3D();
  
    init_gripper_controller();
    
    start();
    return true;
  }

  bool QNode::init(const std::string &master_ip, const std::string &host_ip, QLabel* qLabel_1) 
  {
    std::map<std::string, std::string> remappings;
    remappings["__master"] = "http://"+master_ip+":11311/";
    remappings["__hostname"] = host_ip;

    ros::init(remappings, "trajectory_planning_gui");
    
    if ( ! ros::master::check() ) {
      return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh;

    // rosparam list|grep robot_description_semantic
    // rosparam list|grep robot_description
    if(!nh.hasParam("/robot_description") || !nh.hasParam("/robot_description_semantic"))
    {
      return false;
    }

    // TEACH MODE
    teach_pub = nh.advertise<std_msgs::UInt8>("teach_signal", 1);
    teach_mode_flag = false;

    imageViewLabel_1 = qLabel_1;

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

    // Moveit 
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    // Move group arm
    // arm_move_group = new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP);
    arm_move_group = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP));
    arm_move_group->setPoseReferenceFrame(frame_id_);

    // choose the RRTConnect planner
    // arm_move_group->setPlannerId("RRTConnect");

    kinematic_state = moveit::core::RobotStatePtr(arm_move_group->getCurrentState());
    kinematic_state->setToDefaultValues();

    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(frame_id_));

    // arm_move_group->setStartStateToCurrentState();
    
    // Scaling factors for optionally reducing the maximum joint velocities and
    // accelerations.  Allowed values are in (0,1].  The maximum joint velocity and
    // acceleration specified in the robot model are multiplied by thier respective
    // factors.  If either are outside their valid ranges (importantly, this
    // includes being set to 0.0), the factor is set to the default value of 1.0
    // internally (i.e., maximum joint velocity or maximum joint acceleration).
    
    // arm_move_group->setMaxVelocityScalingFactor(1.0);
    // arm_move_group->setMaxAccelerationScalingFactor(1.0);

    // arm_joint_model_group = arm_move_group->getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
    // ROS_INFO_STREAM(*arm_move_group->getCurrentState());

    const robot_model::RobotModelConstPtr &kmodel = kinematic_state->getRobotModel();
    arm_joint_model_group = kmodel->getJointModelGroup(ARM_PLANNING_GROUP);

    ROS_INFO_NAMED("", "End effector link: %s", arm_move_group->getEndEffectorLink().c_str());

    max_velocity_scaling_factor = 1;
    get_joint_max_velocities();
    setCartParams(10.0, 0.05, 0.0, true, true);

    move_joint_pub_ = nh.advertise<trajectory_planning_gui::MoveJoint>("move_joint", 1);
    init_joint_control();

    jog_frame_pub_ = nh.advertise<trajectory_planning_gui::JogFrame>("jog_frame", 1);
    // Get groups parameter of jog_frame_node
    nh.getParam("/jog_frame_node/group_names", group_names_);
    for (int i=0; i<group_names_.size(); i++)
    {
      ROS_INFO_STREAM("group_names:" << group_names_[i]);
    }
    nh.getParam("/jog_frame_node/link_names", link_names_);
    for (int i=0; i<link_names_.size(); i++)
    {
      ROS_INFO_STREAM("link_names:" << link_names_[i]);
    }

    jog_joint_pub_ = nh.advertise<trajectory_planning_gui::JogJoint>( "jog_joint", 1);
    nh.getParam("/jog_joint_node/joint_names", joint_name_);
    jog_joint_num_ = joint_name_.size();
    jog_value_.resize(jog_joint_num_);

    init_jog_mode();
    init_robot3D();
    
    // Move group gripper
    init_gripper_controller();
    
    start();
    return true;
  }

  void QNode::init_gripper_controller()
  {
    gripper_move_group = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(GRIPPER_PLANNING_GROUP));
    gripper_joint_model_group = gripper_move_group->getCurrentState()->getJointModelGroup(GRIPPER_PLANNING_GROUP);
  }

  void QNode::set_robot_mode(bool sim)
  {
    // sim == true (for simulation mode)
    if(sim)
    {
      sim_mode = true;
      ROS_INFO("SIMULATION MODE ACTIVATED");
    }
    else
    {
      sim_mode = false;
      ROS_INFO("REAL ROBOT CONTROL MODE ACTIVATED");
    }
  }

  bool QNode::get_robot_mode()
  {
    // ros::NodeHandle nh;

    // if (nh.getParam("/mode/sim", sim_mode))
    // {
    //   ROS_INFO_STREAM("sim mode: " << sim_mode);
    // }
    // else 
    // {
    //   ROS_ERROR("Failed to get param '/mode/sim'");
    // }

    return sim_mode;
  }

  void QNode::init_joint_control()
  {
    ros::NodeHandle nh;
    move_joint_sub_ = nh.subscribe("move_joint", 10, &QNode::control_joint_cb, this);
  }

  void QNode::control_joint_cb(trajectory_planning_gui::MoveJointConstPtr msg)
  {
    ROS_INFO("JOINT COMMAND RECEIVED");
    // ROS_INFO_STREAM("number of joints :"<<msg->joint_names.size());

    // Validate the message
    if ((msg->joint_names.size() != msg->positions.size()) || (msg->joint_names.size() != msg->velocities.size()) || (msg->joint_names.size() != msg->accelerations.size()))
    {
      ROS_ERROR("Size mismatch of one or more joint parameters (positions, velocities, accelerations)");
      return;
    }
    
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      actionlib::SimpleClientGoalState state = traj_clients_[controller_name]->getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE)
      {
        return;
      }
    }   

    // Update only if the stamp is older than last_stamp_ + time_from_start
    if (msg->header.stamp > last_stamp_ + ros::Duration(time_from_start_))
    {
      ROS_INFO("start joint state updated");
      // Update reference joint_state
      joint_state_.name.clear();
      joint_state_.position.clear();
      joint_state_.velocity.clear();
      joint_state_.effort.clear();
      for (auto it=joint_map_.begin(); it!=joint_map_.end(); it++)
      {
        joint_state_.name.push_back(it->first);
        joint_state_.position.push_back(it->second);
        joint_state_.velocity.push_back(0.0);
        joint_state_.effort.push_back(0.0);
      }

      // Publish reference joint state for debug
      joint_state_pub_.publish(joint_state_);
    } 

    // Update timestamp of the last jog command
    last_stamp_ = msg->header.stamp;
    
    // Publish trajectory message for each controller
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      auto joint_names_ = it->second.joints;

      trajectory_msgs::JointTrajectory traj;
      trajectory_msgs::JointTrajectoryPoint point;
      point.time_from_start = ros::Duration(time_from_start_);

      for (int i=0; i<joint_names_.size(); i++)
      {
        size_t joint_index = std::distance(msg->joint_names.begin(),
                                          std::find(msg->joint_names.begin(),
                                                    msg->joint_names.end(), 
                                                    joint_names_[i]));
        if (joint_index == msg->joint_names.size())
        {
          ROS_INFO_STREAM("Cannot find joint in jog_joint: " << joint_names_[i]);
          continue;
        }
        size_t state_index = std::distance(joint_state_.name.begin(),
                                          std::find(joint_state_.name.begin(),
                                          joint_state_.name.end(), joint_names_[i]));
        if (state_index == joint_state_.name.size())
        {
          ROS_ERROR_STREAM("Cannot find joint " << joint_names_[i] << " in joint_states_");
          continue;
        }
        // Update start joint position
        joint_state_.position[state_index] = msg->positions[joint_index];
        // Fill joint trajectory joint_names and positions
        traj.joint_names.push_back(joint_names_[i]);
        point.positions.push_back(joint_state_.position[state_index]);
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
      }
      // Fill joint trajectory members
      traj.header.stamp = ros::Time::now();
      traj.header.frame_id = frame_id_;
      traj.points.push_back(point);
      if (use_action_)
      {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = traj;
        traj_clients_[controller_name]->sendGoal(goal);
      }
      else
      {
        traj_pubs_[controller_name].publish(traj);
      }
    }
    // Publish start joint state for debug
    joint_state_pub_.publish(joint_state_);
  }

  void QNode::send_joint_data(std::vector<double> joint_angles, double joint_velocities[6], double joint_accelerations[6])
  {
    // publish frame data
    trajectory_planning_gui::MoveJoint msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;

    int num_joints = joint_names.size();

    msg.joint_names.resize(num_joints);
    msg.positions.resize(num_joints);
    msg.velocities.resize(num_joints);
    msg.accelerations.resize(num_joints);

    for (int i=0; i<num_joints; i++)
    {
      msg.joint_names[i] = joint_names[i];
      msg.positions[i] = joint_angles[i];
      msg.velocities[i] = joint_velocities[i];
      msg.accelerations[i] = joint_accelerations[i];
    }
    move_joint_pub_.publish(msg);
  }

  void QNode::init_robot3D()
  {
    render_panel_ = new rviz::RenderPanel;
    manager_ = new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(),manager_);
    manager_->initialize();
    manager_->startUpdate();
  }

  void QNode::load_robot3D()
  {
    /*
      http://lars.mec.ua.pt/public/LAR%20Projects/Perception/2018_NunoSilva/calibration_gui/calibration_gui/src/gui_myrviz.cpp
    */

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    manager_->setFixedFrame(QString::fromStdString(frame_id_));
    manager_->removeAllDisplays();

    rviz::Display *grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    // grid_->subProp("Line Style")->setValue("Billboards");
    grid_->subProp("Cell Size")->setValue("0");
    ROS_ASSERT( grid_ != NULL );
    
    rviz::Display *robot_ = manager_->createDisplay("rviz/RobotModel", "adjustable robot", false); // (name, Value, Enabled)
    //robot_->subProp("Name")->setValue("RobotModel");
    //robot_->subProp("Value")->setValue("false");
    //robot_->subProp("Enabled")->setValue(true); 
    robot_->subProp("Robot Description")->setValue("robot_description");
    robot_->subProp("TF Prefix")->setValue("");
    robot_->subProp("Visual Enabled")->setValue(true);
    robot_->subProp("Collision Enabled")->setValue(true);//default is false
    robot_->subProp("Alpha")->setValue(1);//default is 1
    robot_->subProp("Update Interval")->setValue(0.2);//default is 0.1  
    robot_->subProp("Links")->subProp("All Links Enabled")->setValue(true);
    robot_->subProp("Links")->subProp("Expand Joint Details")->setValue(false);
    robot_->subProp("Links")->subProp("Expand Link Details")->setValue(false);
    robot_->subProp("Links")->subProp("Expand Tree")->setValue(false);
    robot_->subProp("Links")->subProp("Link Tree Style")->setValue("Links in Alphabetic Order");

    ROS_ASSERT(robot_!=NULL);         

    // Rviz Motion Plan, work in progress
    // rviz::Display *motion_plan_ = manager_->createDisplay("moveit_rviz_plugin/MotionPlanning", "adjustable motionplan", true);
    // ROS_ASSERT(motion_plan_!=NULL);

    // Rviz Planning Scene
    rviz::Display *plan_scene_=manager_->createDisplay("moveit_rviz_plugin/PlanningScene", "adjustable planning",true);
    //plan_scene_->subProp("Enabled")->setValue("true");
    plan_scene_->subProp("Move Group Namespace")->setValue("");
    plan_scene_->subProp("Planning Scene Topic")->setValue("move_group/monitored_planning_scene");
    plan_scene_->subProp("Robot Description")->setValue("robot_description");
    //plan_scene_->subProp("Value")->setValue("true");
    ROS_ASSERT(plan_scene_!=NULL);
  
    /*
    rviz::Display *robot_state_=manager_->createDisplay("moveit_rviz_plugin/RobotState", "adjustable robot state",true);
    ROS_ASSERT(robot_state_!=NULL);

    rviz::Display *robot_traj_=manager_->createDisplay("moveit_rviz_plugin/Trajectory", "adjustable trajectory",true);
    ROS_ASSERT(robot_traj_!=NULL);*/

    /*rviz::Display *mesh_structure_=manager_->createDisplay("rviz/Marker","marker",true);
    ROS_ASSERT(mesh_structure_!=NULL);
    mesh_structure_->subProp("Marker Topic")->setValue("visualization_marker");//this visualization_marker is default
    */

    rviz::Display *marker_array_ = manager_->createDisplay("rviz/MarkerArray","marker array",true);
    marker_array_->subProp("Marker Topic")->setValue("visualization_marker_array");
    marker_array_->subProp("Queue Size")->setValue(100);

    ROS_ASSERT(marker_array_!=NULL);
  }

  rviz::RenderPanel* QNode::attach_rviz_render_panel()
  {
    return render_panel_;
  }

  void QNode::run()
  {
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      updateRobotState();
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("Ros shutdown, proceeding to close the gui.");
    Q_EMIT rosShutdown();
  }

  void QNode::send_teach_mode_signal(int flag)
  {
    // 1  - enable teach mode
    // 0  - disable teach mode
    std_msgs::UInt8 msg;
    msg.data = flag;

    if(flag==1)
    {
      teach_mode_flag = true;
    }
    else
    {
      teach_mode_flag = false;
    }

    teach_pub.publish(msg);
  }

  bool QNode::get_teach_mode_status()
  {
    return teach_mode_flag;
  }

  /**
    This function transform data from target_link_id_ frame to frame_id_ frame.
  **/
  std::vector<double> QNode::frame_transform(const std::string frame_id_, const std::string target_link_id_)
  {
    tf::TransformListener listener_;
    std::vector<double> data;
    data.resize(6);

    tf::StampedTransform transform;
    try
    {
      listener_.waitForTransform(frame_id_, target_link_id_, ros::Time(0), ros::Duration(5.0));
      listener_.lookupTransform(frame_id_, target_link_id_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }  

    data[0] = transform.getOrigin().x();
    data[1] = transform.getOrigin().y();
    data[2] = transform.getOrigin().z();

    // RPY
    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    m.getRPY(data[3], data[4], data[5]);

    // ROS_INFO_STREAM("X: "<<data[0] << " Y: "<<data[1] << " Z: "<<data[2] << "\nROLL: "<<data[3] << " PITCH: "<<data[4] << " YAW: " << data[5]);
    
    return data;
  }

  /**
  * @brief Callback function for the topic jog_joint
  *
  */
  void QNode::jog_joint_cb(trajectory_planning_gui::JogJointConstPtr msg)
  {
    // Validate the message
    if (msg->joint_names.size() != msg->deltas.size())
    {
      ROS_ERROR("Size mismatch of joint_names and deltas");
      return;
    }
    // In intermittent mode, confirm to all of the action is completed
    if (intermittent_)
    {
      for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
      {
        auto controller_name = it->first;
        actionlib::SimpleClientGoalState state = traj_clients_[controller_name]->getState();
        if (state == actionlib::SimpleClientGoalState::ACTIVE)
        {
          return;
        }
      }    
    }
    
    // Update only if the stamp is older than last_stamp_ + time_from_start
    if (msg->header.stamp > last_stamp_ + ros::Duration(time_from_start_))
    {
      ROS_INFO("start joint state updated");
      // Update reference joint_state
      joint_state_.name.clear();
      joint_state_.position.clear();
      joint_state_.velocity.clear();
      joint_state_.effort.clear();
      for (auto it=joint_map_.begin(); it!=joint_map_.end(); it++)
      {
        joint_state_.name.push_back(it->first);
        joint_state_.position.push_back(it->second);
        joint_state_.velocity.push_back(0.0);
        joint_state_.effort.push_back(0.0);
      }

      // Publish reference joint state for debug
      joint_state_pub_.publish(joint_state_);
    }

    // Update timestamp of the last jog command
    last_stamp_ = msg->header.stamp;
    
    // Publish trajectory message for each controller
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      auto joint_names = it->second.joints;

      trajectory_msgs::JointTrajectory traj;
      trajectory_msgs::JointTrajectoryPoint point;
      point.time_from_start = ros::Duration(time_from_start_);

      for (int i=0; i<joint_names.size(); i++)
      {
        size_t jog_index = std::distance(msg->joint_names.begin(),
                                        std::find(msg->joint_names.begin(),
                                        msg->joint_names.end(), joint_names[i]));
        if (jog_index == msg->joint_names.size())
        {
          ROS_INFO_STREAM("Cannot find joint in jog_joint: " << joint_names[i]);
          continue;
        }
        size_t state_index = std::distance(joint_state_.name.begin(),
                                          std::find(joint_state_.name.begin(),
                                          joint_state_.name.end(), joint_names[i]));
        if (state_index == joint_state_.name.size())
        {
          ROS_ERROR_STREAM("Cannot find joint " << joint_names[i] << " in joint_states_");
          continue;
        }
        // Update start joint position
        joint_state_.position[state_index] += msg->deltas[jog_index];
        // Fill joint trajectory joint_names and positions
        traj.joint_names.push_back(joint_names[i]);
        point.positions.push_back(joint_state_.position[state_index]);
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
      }
      // Fill joint trajectory members
      traj.header.stamp = ros::Time::now();
      traj.header.frame_id = frame_id_;
      traj.points.push_back(point);
      if (use_action_)
      {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = traj;
        traj_clients_[controller_name]->sendGoal(goal);
      }
      else
      {
        traj_pubs_[controller_name].publish(traj);
      }
    }

    // Publish start joint state for debug
    joint_state_pub_.publish(joint_state_);
  }

  void QNode::joy_joint_state_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    ROS_INFO("Joystick Joint Control");

    trajectory_planning_gui::JogJoint msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    
    msg.joint_names.resize(6);
    for (int i=0; i<6; i++)
    {
      msg.joint_names[i] = joint_names[i];
    }

    msg.deltas.resize(6);
    
    msg.deltas[0] = 0.05 * (-joy_msg->buttons[4] + joy_msg->buttons[5]);
    msg.deltas[1] = 0.05 * (-joy_msg->buttons[0] + joy_msg->buttons[2]);
    msg.deltas[2] = 0.05 * (joy_msg->buttons[1] - joy_msg->buttons[3]);
    msg.deltas[3] = 0.05 * (joy_msg->axes[7]);
    msg.deltas[4] = 0.05 * (joy_msg->axes[6]);
    msg.deltas[5] = 0.05 * (-joy_msg->buttons[11] + joy_msg->buttons[12]);

    jog_joint_pub_.publish(msg);
  }

  void QNode::joy_frame_state_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    ROS_INFO("Joystick Tool Control");
    // position
    //  x - axis 4 (UP +1, DOWN -1)
    //  y - axis 3 (LEFT +1, RIGHT -1)
    //  z - axis 5 (1{UP} -> -1{DOWN})
       
    // orientation
    //  x - axis 1 (UP +1, DOWN -1)
    //  y - axis 0 (LEFT +1, RIGHT -1)
    //  z - axis 2 (1{UP} -> -1{DOWN})
    
    trajectory_planning_gui::JogFrame msg;
    msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = frame_id_;
    msg.header.frame_id = frame_id_;
    msg.group_name = group_name_;
    msg.link_name = target_link_;
    
    // Position jogging
    if(joy_msg->buttons[joy_map_[joy_name].linear_button])
    {      
      msg.linear_delta.x = joy_map_[joy_name].scale_linear["x"] * joy_msg->axes[joy_map_[joy_name].axis_linear["x"]];
      msg.linear_delta.y = joy_map_[joy_name].scale_linear["y"] * joy_msg->axes[joy_map_[joy_name].axis_linear["y"]];
      msg.linear_delta.z = joy_map_[joy_name].scale_linear["z"] * joy_msg->axes[joy_map_[joy_name].axis_linear["z"]];
      
      // ROS_INFO_STREAM("Linear X: "<<msg.linear_delta.x<<" Linear Y: "<<msg.linear_delta.y<<" Linear Z: "<<msg.linear_delta.z);
    }
    
    // Orientation jogging
    if(joy_msg->buttons[joy_map_[joy_name].angular_button])
    {
      msg.angular_delta.x = joy_map_[joy_name].scale_angular["x"] * joy_msg->axes[joy_map_[joy_name].axis_angular["x"]];
      msg.angular_delta.y = joy_map_[joy_name].scale_angular["y"] * joy_msg->axes[joy_map_[joy_name].axis_angular["y"]];
      msg.angular_delta.z = joy_map_[joy_name].scale_angular["z"] * joy_msg->axes[joy_map_[joy_name].axis_angular["z"]];
    }

    // Publish only if the all command are not equal zero
    // Not good, we need to compare slider value by some way...
    if (msg.linear_delta.x != 0 || msg.linear_delta.y != 0 || msg.linear_delta.z != 0 ||
        msg.angular_delta.x != 0 || msg.angular_delta.y != 0 || msg.angular_delta.z != 0)
    {
      jog_frame_pub_.publish(msg);
    }
    return;
  }

  void QNode::init_joy_jog_mode()
  {
    ros::NodeHandle nh, gnh;

    // ros::param::param<int>("ps4/linear_button", linear_button, 4);
    // ros::param::param<int>("ps4/angular_button", angular_button, 5);

    if (!gnh.hasParam(joy_name))
    {
      ROS_ERROR_STREAM("joystick params not specified.");
      return;
    }
    XmlRpc::XmlRpcValue joy_param_list;
    gnh.getParam(joy_name, joy_param_list);

    if (!joy_param_list.hasMember("linear_button") || !joy_param_list.hasMember("angular_button") || !joy_param_list.hasMember("axis_linear") || !joy_param_list.hasMember("scale_linear") || !joy_param_list.hasMember("axis_angular") || !joy_param_list.hasMember("scale_angular"))
    {
      ROS_ERROR("One or more joystick params missing.");
      return;
    }
    try
    {
      joy_map_[joy_name].linear_button  =  int(joy_param_list["linear_button"]);
      joy_map_[joy_name].angular_button =  int(joy_param_list["angular_button"]);

      std::map<std::string, int> axis_linear;
      axis_linear.insert(std::pair<std::string, int>("x", int(joy_param_list["axis_linear"]["x"])));
      axis_linear.insert(std::pair<std::string, int>("y", int(joy_param_list["axis_linear"]["y"])));
      axis_linear.insert(std::pair<std::string, int>("z", int(joy_param_list["axis_linear"]["z"])));
    
      joy_map_[joy_name].axis_linear = axis_linear; 

      std::map<std::string, double> scale_linear;
      scale_linear.insert(std::pair<std::string, double>("x", double(joy_param_list["scale_linear"]["x"])));
      scale_linear.insert(std::pair<std::string, double>("y", double(joy_param_list["scale_linear"]["y"])));
      scale_linear.insert(std::pair<std::string, double>("z", double(joy_param_list["scale_linear"]["z"])));
    
      joy_map_[joy_name].scale_linear = scale_linear;

      std::map<std::string, int> axis_angular;
      axis_angular.insert(std::pair<std::string, int>("x", int(joy_param_list["axis_angular"]["x"])));
      axis_angular.insert(std::pair<std::string, int>("y", int(joy_param_list["axis_angular"]["y"])));
      axis_angular.insert(std::pair<std::string, int>("z", int(joy_param_list["axis_angular"]["z"])));
    
      joy_map_[joy_name].axis_angular = axis_angular; 

      std::map<std::string, double> scale_angular;
      scale_angular.insert(std::pair<std::string, double>("x", double(joy_param_list["scale_angular"]["x"])));
      scale_angular.insert(std::pair<std::string, double>("y", double(joy_param_list["scale_angular"]["y"])));
      scale_angular.insert(std::pair<std::string, double>("z", double(joy_param_list["scale_angular"]["z"])));
    
      joy_map_[joy_name].scale_angular = scale_angular;

      // https://answers.ros.org/question/266012/getparam-a-nested-stdmap/
      // for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = joy_param_list.begin();
      //     it != joy_param_list.end(); ++it)
      // { 
      //   ROS_INFO_STREAM(it->first); 
      // }
    }
    catch (...)
    {
      ROS_ERROR_STREAM("Error while parsing joystick params");
      return;
    }
    joy_joint_sub = nh.subscribe("joy", 10, &QNode::joy_joint_state_cb, this);
    joy_mode_flag = false;
    // joy_frame_sub = nh.subscribe("joy", 10, &QNode::joy_frame_state_cb, this);
  }

  void QNode::init_joy_joint_control_mode()
  {
    ros::NodeHandle nh;
    if(joy_mode_flag)
    {
      joy_mode_flag = false;
      joy_joint_sub = nh.subscribe("joy", 10, &QNode::joy_joint_state_cb, this);
      
      // http://docs.ros.org/en/api/roscpp/html/classros_1_1TopicManager.html#adbed36c0c18c932e20fb36e2bda5b96b
      joy_frame_sub.shutdown();
    }
  }

  void QNode::init_joy_frame_control_mode()
  {
    ros::NodeHandle nh;
    if(!joy_mode_flag)
    {
      joy_mode_flag = true;
      joy_frame_sub = nh.subscribe("joy", 10, &QNode::joy_frame_state_cb, this);
      joy_joint_sub.shutdown();
    }
  }

  void QNode::disable_joy_control_mode()
  {
    if(joy_mode_flag)
    {
      joy_frame_sub.shutdown();
    }
    else
    {
      joy_joint_sub.shutdown();
    }
  }

  void QNode::init_jog_mode()
  {
    ros::NodeHandle gnh, pnh("~");

    // pnh.param<std::string>("target_link", target_link_, "EEF_Link");
    // pnh.param<std::string>("group", group_name_);
    // pnh.param<double>("time_from_start", time_from_start_, 0.5);
    // pnh.param<bool>("use_action", use_action_, false);
    // pnh.param<bool>("intermittent", intermittent_, false);
    target_link_ = "fts_toolside";
    group_name_ = ARM_PLANNING_GROUP;
    time_from_start_ = 2.0;
    use_action_ = true;
    intermittent_ = false;

    if (not use_action_ && intermittent_)
    {
      ROS_WARN("'intermittent' param should be true with 'use_action'. Assuming to use action'");
      use_action_ = true;
    }
    // Exclude joint list
    gnh.getParam("exclude_joint_names", exclude_joints_);

    if (get_controller_list() < 0)
    {
      ROS_ERROR("get_controller_list faild. Aborted.");
      ros::shutdown();
      return;
    }
    ROS_INFO_STREAM("controller_list:");
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto cinfo = it->second;
      ROS_INFO_STREAM("- " << it->first);
      for (int i=0; i<cinfo.joints.size(); i++)
      {
        ROS_INFO_STREAM("  - " << cinfo.joints[i]);
      }    
    }  

    // Create subscribers
    joint_state_sub_ = gnh.subscribe("joint_states", 10, &QNode::joint_state_cb, this);
    
    jog_frame_sub_ = gnh.subscribe("jog_frame", 10, &QNode::jog_frame_cb, this);
    fk_client_ = gnh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");  
    ik_client_ = gnh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");  
    ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");

    jog_joint_sub_ = gnh.subscribe("jog_joint", 20, &QNode::jog_joint_cb, this);
    // Reference joint_state publisher
    joint_state_pub_ = pnh.advertise<sensor_msgs::JointState>("reference_joint_states", 10);

    if (use_action_)
    {
      // Create action client for each controller
      for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
      {
        auto controller_name = it->first;
        auto controller_info = it->second;
        auto action_name = controller_name + "/" + controller_info.action_ns;
        
        traj_clients_[controller_name] = new arm_control_client(action_name, true);
      }
      for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
      {
        auto controller_name = it->first;
        auto controller_info = it->second;
        auto action_name = controller_name + "/" + controller_info.action_ns;
        
        for(;;)
        {
          if (traj_clients_[controller_name]->waitForServer(ros::Duration(1)))
          {
            ROS_INFO_STREAM(action_name << " is ready.");
            break;
          }
          ROS_WARN_STREAM("Waiting for " << action_name << " server...");
        }
      }
    }
    else
    {
      // Create publisher for each controller
      for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
      {
        auto controller_name = it->first;
        traj_pubs_[controller_name] = gnh.advertise<trajectory_msgs::JointTrajectory>(controller_name + "/command", 10);
      }
    }
  }

  /**
  * @brief Callback function for the topic jog_frame
  * 
  */
  void QNode::jog_frame_cb(trajectory_planning_gui::JogFrameConstPtr msg)
  {
    joint_state_.header.stamp = ros::Time::now();

    // In intermittent mode, confirm to all of the action is completed
    if (intermittent_)
    {
      for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
      {
        auto controller_name = it->first;
        actionlib::SimpleClientGoalState state = traj_clients_[controller_name]->getState();
        if (state == actionlib::SimpleClientGoalState::ACTIVE)
        {
          return;
        }
      }    
    }
    // Update reference frame only if the stamp is older than last_stamp_ + time_from_start_
    if (msg->header.stamp > last_stamp_ + ros::Duration(time_from_start_))
    {
      joint_state_.name.clear();
      joint_state_.position.clear();
      joint_state_.velocity.clear();
      joint_state_.effort.clear();

      for (auto it=joint_map_.begin(); it!=joint_map_.end(); it++)
      {
        // Exclude joint in exclude_joints_
        if (std::find(exclude_joints_.begin(),
                      exclude_joints_.end(), it->first) != exclude_joints_.end())
        {
          ROS_INFO_STREAM("joint " << it->first << "is excluded from FK");
          continue;
        }
        // Update reference joint_state
        joint_state_.name.push_back(it->first);
        joint_state_.position.push_back(it->second);
        joint_state_.velocity.push_back(0.0);
        joint_state_.effort.push_back(0.0);
      }

      // Update forward kinematics
      moveit_msgs::GetPositionFK fk;

      fk.request.header.frame_id = msg->header.frame_id;
      fk.request.header.stamp = ros::Time::now();
      fk.request.fk_link_names.clear();
      fk.request.fk_link_names.push_back(msg->link_name);
      fk.request.robot_state.joint_state = joint_state_;

      if (fk_client_.call(fk))
      {
        if(fk.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          ROS_INFO_STREAM("fk: " << fk.request);
          ROS_WARN("****FK error %d", fk.response.error_code.val);
          return;
        }
        if (fk.response.pose_stamped.size() != 1)
        {
          for (int i=0; i<fk.response.pose_stamped.size(); i++)
          {
            ROS_ERROR_STREAM("fk[" << i << "]:\n" << fk.response.pose_stamped[0]);
          }
        }
        pose_stamped_ = fk.response.pose_stamped[0];
      }
      else
      {
        ROS_ERROR("Failed to call service /compute_fk");
        return;
      }
    }
    // Update timestamp of the last jog command
    last_stamp_ = msg->header.stamp;

    // Solve inverse kinematics
    moveit_msgs::GetPositionIK ik;

    ik.request.ik_request.group_name = msg->group_name;
    ik.request.ik_request.ik_link_name = msg->link_name;
    ik.request.ik_request.robot_state.joint_state = joint_state_;
    ik.request.ik_request.avoid_collisions = msg->avoid_collisions;
    
    geometry_msgs::Pose act_pose = pose_stamped_.pose;
    geometry_msgs::PoseStamped ref_pose;
    
    ref_pose.header.frame_id = msg->header.frame_id;
    ref_pose.header.stamp = ros::Time::now();
    ref_pose.pose.position.x = act_pose.position.x + msg->linear_delta.x;
    ref_pose.pose.position.y = act_pose.position.y + msg->linear_delta.y;
    ref_pose.pose.position.z = act_pose.position.z + msg->linear_delta.z;

    // Apply orientation jog
    tf::Quaternion q_ref, q_act, q_jog;
    tf::quaternionMsgToTF(act_pose.orientation, q_act);
    double angle = sqrt(msg->angular_delta.x*msg->angular_delta.x +
                        msg->angular_delta.y*msg->angular_delta.y +
                        msg->angular_delta.z*msg->angular_delta.z);
    tf::Vector3 axis(1,0,0);
    if (fabs(angle) < DBL_EPSILON)
    {
      angle = 0.0;
    }
    else
    {
      axis.setX(msg->angular_delta.x/angle);
      axis.setY(msg->angular_delta.y/angle);
      axis.setZ(msg->angular_delta.z/angle);
    }
    ROS_INFO_STREAM("axis: " << axis.x() << ", " << axis.y() << ", " << axis.z());
    ROS_INFO_STREAM("angle: " << angle);

    q_jog.setRotation(axis, angle);
    q_ref = q_jog*q_act;
    quaternionTFToMsg(q_ref, ref_pose.pose.orientation);

    ik.request.ik_request.pose_stamped = ref_pose;

    ROS_INFO_STREAM("ik:\n" << ik.request);

    if (!ik_client_.call(ik))
    {
      ROS_ERROR("Failed to call service /compute_ik");
      return;
    }
    if (ik.response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_WARN("****IK error %d", ik.response.error_code.val);
      return;
    }

    ROS_INFO_STREAM("ik response:\n" << ik.response);
    
    auto state = ik.response.solution.joint_state;
    geometry_msgs::PoseStamped pose_check;
    
    // Make sure the solution is valid in joint space
    double error = 0;
    for (int i=0; i<state.name.size(); i++)
    {
      for (int j=0; j<joint_state_.name.size(); j++)
      {
        if (state.name[i] == joint_state_.name[j])
        {
          double e = fabs(state.position[i] - joint_state_.position[j]);
          if (e > error)
          {
            error = e;
          }
          break;
        }
      }
    }
    if (error > M_PI / 2)
    {
      ROS_ERROR_STREAM("**** Validation check Failed: " << error);
      return;
    }

    // Publish trajectory message for each controller
    for (auto it=cinfo_map_.begin(); it!=cinfo_map_.end(); it++)
    {
      auto controller_name = it->first;
      auto joint_names = it->second.joints;

      std::vector<double> positions, velocities, accelerations;

      positions.resize(joint_names.size());
      velocities.resize(joint_names.size());
      accelerations.resize(joint_names.size());

      for (int i=0; i<joint_names.size(); i++)
      {
        size_t index = std::distance(state.name.begin(),
                                     std::find(state.name.begin(),
                                     state.name.end(), joint_names[i]));
        if (index == state.name.size())
        {
          ROS_WARN_STREAM("Cannot find joint " << joint_names[i] << " in IK solution");        
        }
        positions[i] = state.position[index];
        velocities[i] = 0;
        accelerations[i] = 0;
      }
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = positions;
      point.velocities = velocities;
      point.accelerations = accelerations;
      point.time_from_start = ros::Duration(time_from_start_);

      if (use_action_)
      {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.header.frame_id = frame_id_;
        goal.trajectory.joint_names = joint_names;
        goal.trajectory.points.push_back(point);

        traj_clients_[controller_name]->sendGoal(goal);
      }
      else
      {
        trajectory_msgs::JointTrajectory traj;
        traj.header.stamp = ros::Time::now();
        traj.header.frame_id = frame_id_;
        traj.joint_names = joint_names;
        traj.points.push_back(point);
        
        traj_pubs_[controller_name].publish(traj);
      }
    }
    // update pose_stamped_
    pose_stamped_.pose = ref_pose.pose;
  }

  int QNode::get_controller_list()
  {
    ros::NodeHandle gnh;

    // Get controller information from move_group/controller_list
    if (!gnh.hasParam("move_group/controller_list"))
    {
      ROS_ERROR_STREAM("move_group/controller_list is not specified.");
      return -1;
    }
    XmlRpc::XmlRpcValue controller_list;
    gnh.getParam("move_group/controller_list", controller_list);
    // ROS_INFO_STREAM("# of controllers: "<<controller_list.size());
    // excluding gripper controller for now from controller list
    // that means including only arm controller
    // for (int i = 0; i < (controller_list.size() - 1); i++)
    // Below code includes all available controllers
    for (int i = 0; i < controller_list.size(); i++)
    {
      if (!controller_list[i].hasMember("name"))
      {
        ROS_ERROR("name must be specifed for each controller.");
        return -1;
      }
      if (!controller_list[i].hasMember("joints"))
      {
        ROS_ERROR("joints must be specifed for each controller.");
        return -1;
      }
      try
      {
        // get name member
        std::string name = std::string(controller_list[i]["name"]);
        // ROS_INFO_STREAM("controller name: "<<name);
        // get action_ns member if exists
        std::string action_ns = std::string("");
        if (controller_list[i].hasMember("action_ns"))
        {
          action_ns = std::string(controller_list[i]["action_ns"]);
        }
        // get joints member
        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR_STREAM("joints for controller " << name << " is not specified as an array");
          return -1;
        }
        auto joints = controller_list[i]["joints"];
        // Get type member
        std::string type = std::string("FollowJointTrajectory");
        if (!controller_list[i].hasMember("type"))
        {
          ROS_WARN_STREAM("type is not specifed for controller " << name << ", using default FollowJointTrajectory");
        }
        type = std::string(controller_list[i]["type"]);
        if (type != "FollowJointTrajectory")
        {
          ROS_ERROR_STREAM("controller type " << type << " is not supported");
          return -1;
        }
        // Create controller map
        cinfo_map_[name].action_ns = action_ns;
        cinfo_map_[name].joints.resize(joints.size());
        for (int j = 0; j < cinfo_map_[name].joints.size(); ++j)
        {
          cinfo_map_[name].joints[j] = std::string(joints[j]);
        }
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Caught unknown exception while parsing controller information");
        return -1;
      }
    }
    return 0;
  }

  /**
  * @brief Callback function for the topic joint_state
  *
  */
  void QNode::joint_state_cb(sensor_msgs::JointStateConstPtr msg)
  {
    // Check that the msg contains joints
    if (msg->name.empty() || msg->name.size() != msg->position.size())
    {
      ROS_WARN("Invalid JointState message");
      return;
    }
    // Update joint information
    for (int i=0; i<msg->name.size(); i++)
    {
      joint_map_[msg->name[i]] = msg->position[i];
    }
  }

  void QNode::publish_jog_frame_data(const trajectory_planning_gui::JogFrame msg)
  {
    jog_frame_pub_.publish(msg);
  }

  void QNode::publish_jog_joint_data(const trajectory_planning_gui::JogJoint msg)
  {
    jog_joint_pub_.publish(msg);
  }

  void QNode::setCartParams(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_)
  {
    /*! Set the necessary parameters for the MoveIt and the Cartesian Path Planning.
        These parameters correspond to the ones that the user has entered or the default ones before the execution of the Cartesian Path Planner.
    */
    ROS_INFO_STREAM("MoveIt and Cartesian Path parameters from UI:\n MoveIt Plan Time:"<<plan_time_
                    <<"\n Cartesian Path Step Size:"<<cart_step_size_
                    <<"\n Jump Threshold:"<<cart_jump_thresh_
                    <<"\n Replanning:"<<moveit_replan_
                    <<"\n Avoid Collisions:"<<avoid_collisions_);

    PLAN_TIME_        = plan_time_;
    MOVEIT_REPLAN_    = moveit_replan_;
    CART_STEP_SIZE_   = cart_step_size_;
    CART_JUMP_THRESH_ = cart_jump_thresh_;
    AVOID_COLLISIONS_ = avoid_collisions_;
  }

  void QNode::update_max_vel_scale_factor(double value)
  {
    max_velocity_scaling_factor = value;
    ROS_INFO_STREAM("Updated max velocity scale factor to :"<<max_velocity_scaling_factor);
  }

  void QNode::set_camera_id(int id=0)
  {
    camera_id = id;
    change_cam_subscriber(camera_id);
  }

  void QNode::init_cam_subscriber()
  {
    ros::NodeHandle nh;
    image_sub = nh.subscribe("/camera/rgb/image_raw", 1, &QNode::robotCamViewer, this);
  }

  void QNode::change_cam_subscriber(int camera_id)
  {
    image_sub.shutdown();
    ros::NodeHandle nh;
    if(camera_id==0)
    {
      image_sub = nh.subscribe("/camera/rgb/image_raw", 1, &QNode::robotCamViewer, this);
    }
    else
    {
      image_sub = nh.subscribe("/camera/rgb/image_raw", 1, &QNode::robotCamViewer, this);
    } 
  }
  
  void QNode::robotCamViewer(const sensor_msgs::ImageConstPtr& msg)
  {
    imageViewLabel_1->setPixmap(rgb_msg_to_pixmap(msg));
  }
  
  QPixmap QNode::rgb_msg_to_pixmap(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    QImage image;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // cv::Size size = cv_ptr->image.size();

      QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
      image = temp;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return(QPixmap::fromImage(image));
  }

  // void QNode::robotCamViewerLower(const sensor_msgs::ImageConstPtr& msg)
  // {
  //   imageViewLabel_2->setPixmap(rgb_msg_to_pixmap_lower(msg));
  // }
  
  // QPixmap QNode::rgb_msg_to_pixmap_lower(const sensor_msgs::ImageConstPtr& msg)
  // {
  //   cv_bridge::CvImagePtr cv_ptr;
  //   try
  //   {
  //     cv_ptr = cv_bridge::toCvCopy(msg);

  //     // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
  //     // QImage temp(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);

  //     // QImage image;
  //     // image = temp;

  //     // return(QPixmap::fromImage(image));
  //   }
  //   catch (cv_bridge::Exception& e)
  //   {
  //     ROS_ERROR("cv_bridge exception: %s", e.what());
  //   }

  //   //Copy the image.data to imageBuf.
  //   cv::Mat mono8_img = cv::Mat(cv_ptr->image.size(), CV_16UC1);
  //   cv::convertScaleAbs(cv_ptr->image, mono8_img, 100, 0.0);

  //   QImage temp(&(mono8_img.data[0]), msg->width, msg->height, QImage::Format_RGB888);

  //   QImage image;
  //   image = temp;

  //   return(QPixmap::fromImage(image));
  // }

  /*!
  * \brief Convert radians to degrees
  */
  double QNode::to_degrees(double radians)
  {
    return radians * 180.0 / M_PI;
  }

  /*!
  * \brief Convert degrees to radians
  */

  double QNode::from_degrees(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  void QNode::set_reference_frame(const std::string frame_id)
  {
    frame_id_ = frame_id;
    ROS_INFO_STREAM("REFERENCE FRAME: "<<frame_id_);
  }
  
  /**
    update the current joint angles and end effector pose of the arm 
    w.r.t. reference frame 'frame_id_' 
  **/
  void QNode::updateRobotState()
  {
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    std::vector<double> jointValues = arm_move_group->getCurrentJointValues();
    // std::vector<double> jointValues2 = move_group2_->getCurrentJointValues();
    std::vector<double> temp_angle;
    temp_angle.push_back(jointValues.at(0));
    temp_angle.push_back(jointValues.at(1));
    temp_angle.push_back(jointValues.at(2));
    temp_angle.push_back(jointValues.at(3));
    temp_angle.push_back(jointValues.at(4));
    temp_angle.push_back(jointValues.at(5));
    // temp_angle.push_back(jointValues2.at(0));
    present_joint_angle_ = temp_angle;

    geometry_msgs::Pose current_pose = arm_move_group->getCurrentPose(arm_move_group->getEndEffectorLink()).pose;  
    
    // std::vector<double> temp_position;
    // temp_position.push_back(current_pose.position.x);
    // temp_position.push_back(current_pose.position.y);
    // temp_position.push_back(current_pose.position.z);

    // tf::Quaternion q(
    //   current_pose.orientation.x,
    //   current_pose.orientation.y,
    //   current_pose.orientation.z,
    //   current_pose.orientation.w
    // );

    // tf::Matrix3x3 mat(q);
    // double roll, pitch, yaw;
    // mat.getRPY(roll, pitch, yaw);

    // temp_position.push_back(roll);
    // temp_position.push_back(pitch);
    // temp_position.push_back(yaw);

    // present_kinematics_position_ = temp_position;

    present_kinematics_position_ = frame_transform(frame_id_, target_link_);
  }

  tf::Quaternion QNode::RPYToQuaternion(float R, float P, float Y)
  {
    tf::Matrix3x3 mat;
    mat.setEulerYPR(Y,P,R);

    tf::Quaternion quat;
    mat.getRotation(quat);

    return quat;
  }

  geometry_msgs::Vector3 QNode::QuaternionToRPY(const geometry_msgs::Quaternion msg)
  {
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    return rpy;
  }

  void QNode::get_joint_max_velocities()
  {
    /**
      Get the maximum velocities of joints defined inside joint_limits.yaml file
      *** owr_moveit_config/config/joint_limits.yaml *** 
    **/

    max_vel.resize(joint_names.size());
    for(int i=0;i<joint_names.size();i++)
    {
      ros::param::get("/robot_description_planning/joint_limits/"+joint_names[i]+"/max_velocity", max_vel[i]);
    }

    ROS_INFO("*************** MAXIMUM JOINT VELOCITIES *****************");
    ROS_INFO_STREAM("[BJ: " << max_vel[0] << " SJ: " << max_vel[1] << " EJ: " <<  max_vel[2] << " W1J: " << max_vel[3] << " W2J: " << max_vel[4] << " W3J: " << max_vel[5] <<"]");
  }

  bool QNode::checkWayPointValidity(const geometry_msgs::Pose& waypoint)
  {
    /*
      checks if the Way-Point is within the valid IK solution for the Robot.
      In the case when a point is outside the valid IK solution this function 
      send a signal to the Qt Gui.
    */
    
    ROS_INFO_STREAM("x: "<< waypoint.position.x << " y: " << waypoint.position.y << " z: " << waypoint.position.z);
    

    bool found_ik = kinematic_state->setFromIK(arm_joint_model_group, waypoint, 3, 0.005);

    if(found_ik)
    {
      ROS_INFO("IK solution found for above point!");
      return true;
    }
    ROS_INFO("IK solution not found!");
    return false;
  }

  bool QNode::setRandomPose()
  {
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    arm_move_group->setRandomTarget();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // set maximum time to find a plan
    // arm_move_group->setPlanningTime(10.0);
    bool success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success == false)
      return false;

    // if (!success)
    //   throw std::runtime_error("No plan found");
  
    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    // Execute the plan
    ros::Time start = ros::Time::now();

    arm_move_group->move();

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    spinner.stop();
    return true;
  }

  bool QNode::setHomePose()
  {
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    arm_move_group->setNamedTarget("HOME");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success == false)
      return false;

    arm_move_group->move();

    spinner.stop();
    return true;
  }

  bool QNode::setInitPose()
  {
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    arm_move_group->setNamedTarget("INIT");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success == false)
      return false;

    arm_move_group->move();

    spinner.stop();
    return true;
  }

  bool QNode::setUpPose()
  {
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    arm_move_group->setNamedTarget("UP");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success == false)
      return false;

    arm_move_group->move();

    spinner.stop();
    return true;
  }

  std::vector<double> QNode::getPresentJointAngle()
  {
    return present_joint_angle_;
  }

  std::vector<double> QNode::getPresentKinematicsPosition()
  {
    return present_kinematics_position_;
  }

  int QNode::get_max_vel_scale_factor(void)
  {
    return max_velocity_scaling_factor*100;
  }

  bool QNode::setJointSpacePath(std::vector<double> joint_angle)
  {
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    // Next get the current set of joint values for the group.
    // const robot_state::JointModelGroup* joint_model_group =
    //     arm_move_group->getCurrentState()->getJointModelGroup("arm_manipulator");
        
    moveit::core::RobotStatePtr current_state = arm_move_group->getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(arm_joint_model_group, joint_group_positions);

    joint_group_positions[0] = joint_angle.at(0);  // radians
    joint_group_positions[1] = joint_angle.at(1);  // radians
    joint_group_positions[2] = joint_angle.at(2);  // radians
    joint_group_positions[3] = joint_angle.at(3);  // radians
    joint_group_positions[4] = joint_angle.at(4);  // radians
    joint_group_positions[5] = joint_angle.at(5);  // radians
    arm_move_group->setJointValueTarget(joint_group_positions);
    // arm_move_group->setPlanningTime(10.0);

    // ROS_INFO("base %f", joint_group_positions[0]);
    // ROS_INFO("shoulder %f", joint_group_positions[1]);
    // ROS_INFO("elbow %f", joint_group_positions[2]);
    // ROS_INFO("wrist1 %f", joint_group_positions[3]);
    // ROS_INFO("wrist2 %f", joint_group_positions[4]);
    // ROS_INFO("wrist3 %f", joint_group_positions[5]);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success == false)
      return false;

    arm_move_group->move();
    return true;
  }

  bool QNode::setTaskSpacePath(std::vector<double> kinematics_pose)
  {
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    // Uncomment below to keep the end-effector parallel to the ground
    /*
    moveit_msgs::OrientationConstraint oc;
    oc.link_name = "end_effector_link";
    oc.header.frame_id = "link1";
    oc.orientation.w = 1.0;
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 3.14;
    oc.weight = 1.0;
    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(oc);
    arm_move_group->setPathConstraints(constraints);
    */

    geometry_msgs::Pose target_pose;
    target_pose.position.x = kinematics_pose.at(0);
    target_pose.position.y = kinematics_pose.at(1);
    target_pose.position.z = kinematics_pose.at(2);

    tf::Quaternion qt = RPYToQuaternion(kinematics_pose.at(3), kinematics_pose.at(4), kinematics_pose.at(5));

    // ROS_INFO("x %f", target_pose.position.x);
    // ROS_INFO("y %f", target_pose.position.y);
    // ROS_INFO("z %f", target_pose.position.z);
    // ROS_INFO("\nroll %f", kinematics_pose.at(3));
    // ROS_INFO("pitch %f", kinematics_pose.at(4));
    // ROS_INFO("yaw %f", kinematics_pose.at(5));

    target_pose.orientation.w = qt.getW();
    target_pose.orientation.x = qt.getX();
    target_pose.orientation.y = qt.getY();
    target_pose.orientation.z = qt.getZ();
    // arm_move_group->setGoalOrientationTolerance(0.05);
    // arm_move_group->setGoalPositionTolerance(0.05);
    
    // set starting pose
    arm_move_group->setStartStateToCurrentState();
    // arm_move_group->setPlanningTime(10.0);
    // set target pose
    arm_move_group->setPoseTarget(target_pose); 
    
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool success = static_cast<bool>(arm_move_group->plan(arm_plan));
    ROS_INFO("Visualizing plan to target: %s", success ? "SUCCEEDED" : "FAILED");
    
    if (success == false)
      return false;

    arm_move_group->execute(arm_plan);

    spinner.stop();
    return true;
  }

  // Create a ROS action client to move OW arm
  void QNode::createArmClient(arm_control_client_Ptr& actionClient)
  {
    ROS_INFO("Creating action client to arm_manipulator controller ...");

    actionClient.reset(new arm_control_client("/arm_manipulator_controller/follow_joint_trajectory"));

    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while(!actionClient -> waitForServer(ros::Duration(5.0)) && ros::ok() && iterations < max_iterations)
    {
      ROS_DEBUG("Waiting for the arm_manipulator_controller action server to come up");
      ++iterations;
    }
    if(iterations == max_iterations)
      throw std::runtime_error("Error in createArmClient: arm manipulator controller action server not available");
  }

  // Generates a simple trajectory with two waypoints to move OW arm 
  void QNode::waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal, int iterations, std::vector<std::vector<double>> waypoints, double joint_velocities[], double joint_accelerations[])
  {
      // The joint names, which apply to all waypoints
      ROS_INFO_STREAM("#joints: "<<joint_names.size());
      for(int i=0;i<joint_names.size();i++)
      {
        goal.trajectory.joint_names.push_back(joint_names[i]);
      }

      // number of waypoints added
      int n = waypoints.size();
      
      // total number of waypoints in this goal trajectory = n*iterations
      // this will be equal to n in case of single iteration
      goal.trajectory.points.resize(n*iterations);

      double time = 0.0;
      for(int count = 0; count < iterations; ++count)
      {
        std::vector<double> waypoint;
        for(int index = 0; index < n; ++index)
        {
          std::vector<double> duration;
          duration.resize(joint_names.size());

          // get current joint angles(in radians)
          // std::vector<double> curr_pos = getPresentJointAngle();
          waypoint = waypoints[index];

          for(int i=0;i<joint_names.size();i++)
          {
            // duration[i] = std::max(std::abs(waypoint[i]-curr_pos[i]) / max_vel[i], min_traj_dur);
            duration[i] = std::max(std::abs(waypoint[i]-initial_pos[i]) / joint_velocities[i], min_traj_dur);
            initial_pos[i] = waypoint[i];
          }

          time += *std::max_element(duration.begin(), duration.end()) / max_velocity_scaling_factor;
          
          // Positions
          goal.trajectory.points[n*count+index].positions.resize(6);
          for(int i = 0; i < 6; ++i)
          {
            goal.trajectory.points[n*count+index].positions[i] = waypoint[i];
          }
          
          ROS_INFO_STREAM("[BJ: " << waypoint[0] << " SJ: " << waypoint[1] << " EJ: " <<  waypoint[2] << " W1J: " << waypoint[3] << " W2J: " << waypoint[4] << " W3J: " << waypoint[5] <<"]");
          ROS_INFO_STREAM("PLAN TIME (from start): " << time);

          // Velocities
          // goal.trajectory.points[n*count+index].velocities.resize(6);
          // for (int i = 0; i < 6; ++i)
          // {
          //     goal.trajectory.points[n*count+index].velocities[i] = joint_velocities[i];
          // }

          // Accelerations
          // goal.trajectory.points[n*count+index].accelerations.resize(6);
          // for (int i = 0; i < 6; ++i)
          // {
          //     goal.trajectory.points[n*count+index].accelerations[i] = joint_accelerations[i];
          // }

          // ROS_INFO("Time from start %f", time);
          // To be reached 'time' second after starting along the trajectory
          goal.trajectory.points[n*count+index].time_from_start = ros::Duration(time);
        }
      }
  }

  void QNode::call_query_state_client()
  {
    ros::NodeHandle nh;

    query_state_client = nh.serviceClient<control_msgs::QueryTrajectoryState>
                                            ("/arm_manipulator_controller/query_state");
      
    control_msgs::QueryTrajectoryState srv;
    srv.request.time.sec = 1.0;

    if(query_state_client.call(srv))
    {
      initial_pos = srv.response.position;
    }
    else
    {
      ROS_FATAL("Something wrong with the server response!!");
    }
  }

  void QNode::run_trajectory(int iterations, std::vector<std::vector<double>> waypoints, double joint_velocities[], double joint_accelerations[])
  { 
      ros::AsyncSpinner spinner(1); 
      spinner.start();

      ROS_INFO("Starting joint_trajectory_control application ...");
      ROS_INFO("itertaions %d", iterations);

      // Precondition: Valid clock
      // ros::NodeHandle nh;
      if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
      {
          ROS_FATAL("Timed-out waiting for valid time.");
          // return 1;
      }

      // Create an arm controller action client to move the OW arm
      createArmClient(ArmClient);

      // Generates the goal for the OW arm
      control_msgs::FollowJointTrajectoryGoal arm_goal;
      waypoints_arm_goal(arm_goal, iterations, waypoints, joint_velocities, joint_accelerations);
      // Sends the command to start the given trajectory 1s from now
      arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
      ArmClient->sendGoal(arm_goal);

      QFuture<void> future = QtConcurrent::run(this, &QNode::signal_on_trajectory_completion, ArmClient);
      
      // while(!(ArmClient->getState().isDone()) && ros::ok())
      // {
      //     ros::Duration(6).sleep(); // sleep for 6 seconds
      // }
      spinner.stop();
  }

  void QNode::cancel_trajectory()
  {
    // http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1ActionClient.html
    // Cancel all goals currently running on the action server. 
    ArmClient->cancelAllGoals();
    Q_EMIT trajectoryPlanCancelled();
  }

  void QNode::pause_trajectory()
  {
    stop_pos = present_joint_angle_;
    // 	Cancel the goal that the arm is currently pursuing.
    ArmClient->cancelGoal();
    
    // Q_EMIT trajectoryPlanPaused();

  }

  void QNode::signal_on_trajectory_completion(arm_control_client_Ptr& actionClient)
  {
    while(ros::ok())
    {
      // ROS_INFO_STREAM("arm client result _> " <<actionClient->getResult());
      // ROS_INFO_STREAM("arm client _> " <<actionClient->getState().isDone());
      if(actionClient->getState().isDone()) // --> 1
      {
        // Q_EMIT trajectoryExecutionFinished();
        Q_EMIT cartesianPathExecuteFinished();
        return;
      }
      
    }  
  }

  geometry_msgs::Pose QNode::parseWayPoint(const tf::Transform& point_pos)
  {
    geometry_msgs::Pose waypoint;
    tf::poseTFToMsg(point_pos, waypoint);
    return waypoint;
  }

  std::vector<geometry_msgs::Pose> QNode::parseWayPoints(std::vector<tf::Transform> waypoints_pos)
  {
    /*! 
      Get the vector of all Way-Points and convert it to geometry_msgs::Pose.
    */
    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> waypoints;

    // we need to change all the positions and orientations vectors to geometry_msgs::Pose
    for(int i=0;i<waypoints_pos.size();i++)
    {
      tf::poseTFToMsg (waypoints_pos[i], target_pose);
      waypoints.push_back(target_pose);
    }
    return waypoints;
  }

  void QNode::delete_visual_tools()
  {
    visual_tools_->deleteAllMarkers();
    ros::Duration(0.1).sleep();
  }

  void QNode::visualize_path_on_load(std::vector<tf::Transform> waypoints_pos)
  {
    visual_tools_->publishPath(parseWayPoints(waypoints_pos), rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    visual_tools_->trigger();
    ros::Duration(0.5).sleep();
  }

  void QNode::execute_cartesian_path(std::vector<geometry_msgs::Pose> waypoints)
  {
    Q_EMIT cartesianPathExecuteStarted();
    /**
    for (std::vector<geometry_msgs::Pose>::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i){
          std::cout << *i;
    }**/

    arm_move_group->setPlanningTime(PLAN_TIME_);
    arm_move_group->allowReplanning(MOVEIT_REPLAN_);

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit_msgs::RobotTrajectory trajectory_;
    double fraction = arm_move_group->computeCartesianPath(waypoints, CART_STEP_SIZE_, CART_JUMP_THRESH_, trajectory_, AVOID_COLLISIONS_);

    robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(), ARM_PLANNING_GROUP);
    rt.setRobotTrajectoryMsg(*kinematic_state, trajectory_);

    ROS_INFO_STREAM("Pose reference frame: " << arm_move_group->getPoseReferenceFrame());

  	bool success = iptp.computeTimeStamps(rt);
  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_);
    // Finally plan and execute the trajectory
  	arm_plan.trajectory_ = trajectory_;
    // ROS_INFO_STREAM("Robot Trajectory \n" <<trajectory_);
  	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0); 
    Q_EMIT cartesianPathCompleted(fraction); 

    // visual_tools_->publishTrajectoryLine(trajectory_, arm_joint_model_group, 0.01, true);
   
  	arm_move_group->execute(arm_plan);

    kinematic_state = arm_move_group->getCurrentState();
    Q_EMIT cartesianPathExecuteFinished();

    // /*! Update the joint model group so we have more realistic update of the way-points while user updates their pose */
    // const robot_model::RobotModelConstPtr &kmodel = kinematic_state->getRobotModel();
    // joint_model_group = kmodel->getJointModelGroup("manipulator");
  }

  void QNode::cartesianPathHandler(std::vector<tf::Transform> waypoints_pos)
  {
    /*! Since the execution of the Cartesian path is time consuming and can lead to locking 
        up of the Plugin and the RViz enviroment the function for executing the Cartesian Path 
        Plan has been placed in a separtate thread. This prevents the RViz and the Plugin to lock.
    */

    std::vector<geometry_msgs::Pose> waypoints = parseWayPoints(waypoints_pos);

    ROS_INFO("Starting concurrent process for Cartesian Path");
    QFuture<void> future = QtConcurrent::run(this, &QNode::execute_cartesian_path, waypoints);
  }

  void QNode::waypoints_straight(std::vector<geometry_msgs::Pose> &waypoints, const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose)
  {
    if(sqrt(pow(goal_pose.position.x - start_pose.position.x, 2.0) + pow(goal_pose.position.y - start_pose.position.y, 2.0) + pow(goal_pose.position.z - start_pose.position.z, 2.0)) < 0.05)
    {
      return;
    }
    geometry_msgs::Pose pose;

    geometry_msgs::Vector3 start_pose_rpy = QuaternionToRPY(start_pose.orientation);
    geometry_msgs::Vector3 goal_pose_rpy = QuaternionToRPY(goal_pose.orientation);

    double roll, pitch, yaw;
    roll = (start_pose_rpy.x + goal_pose_rpy.x)/2;
    pitch = (start_pose_rpy.y + goal_pose_rpy.y)/2;
    yaw = (start_pose_rpy.z + goal_pose_rpy.z)/2;

    tf::Quaternion qt = RPYToQuaternion(roll, pitch, yaw);
    pose.orientation.w = qt.getW();
    pose.orientation.x = qt.getX();
    pose.orientation.y = qt.getY();
    pose.orientation.z = qt.getZ();
    pose.position.x = (start_pose.position.x + goal_pose.position.x)/2;
    pose.position.y = (start_pose.position.y + goal_pose.position.y)/2;
    pose.position.z = (start_pose.position.z + goal_pose.position.z)/2;

    waypoints_straight(waypoints, start_pose, pose);
    waypoints.push_back(pose);
    waypoints_straight(waypoints, pose, goal_pose);
  }

  void QNode::waypoints_arc(std::vector<geometry_msgs::Pose> &waypoints, const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose, float angle_resolution)
  {
    double d_angle = angle_resolution*3.14/180;
    double angle= 0;
    geometry_msgs::Pose ee_point_goal; //end_effector_trajectory

    geometry_msgs::Pose mid_pose;
    mid_pose.position.x =(start_pose.position.x + goal_pose.position.x)/2;
    mid_pose.position.y = (start_pose.position.y + goal_pose.position.y)/2;
    mid_pose.position.z = (start_pose.position.z + goal_pose.position.z)/2;
    // mid_pose.orientation = goal_pose.orientation
    double radius = 0.5*sqrt(pow(goal_pose.position.x - start_pose.position.x, 2.0) + pow(goal_pose.position.y - start_pose.position.y, 2.0) + pow(goal_pose.position.z - start_pose.position.z, 2.0));

    for (int i=0; i<(180/angle_resolution); i++)
    {
      //discretize the trajectory
      angle += d_angle;
      ee_point_goal.position.x = mid_pose.position.x + radius*cos(angle);
      ee_point_goal.position.y = mid_pose.position.y + radius*sin(angle);
      ee_point_goal.position.z = mid_pose.position.z; 
      ee_point_goal.orientation = goal_pose.orientation;
      waypoints.push_back(ee_point_goal);
    }
  }

  void QNode::cartesian_curve(std::vector<tf::Transform> waypoints_pos, int index, float angle_resolution)
  {
    std::vector<geometry_msgs::Pose> end_points = parseWayPoints(waypoints_pos);
    std::vector<geometry_msgs::Pose> waypoints;
    if(index==0)
    {
      waypoints.push_back(end_points[0]);
      waypoints_straight(waypoints, end_points[0], end_points[1]);
      waypoints.push_back(end_points[1]);
    }
    else if(index==1)
    {
      waypoints.push_back(end_points[0]);
      waypoints_arc(waypoints, end_points[0], end_points[1], angle_resolution);
      waypoints.push_back(end_points[1]);
    }
    for(int i=0; i<waypoints.size(); i++)
    {
      ROS_INFO_STREAM("x: "<< waypoints[i].position.x << " y: " << waypoints[i].position.y << " z: " << waypoints[i].position.z);
    }
    addMarker(waypoints);
  }

  void QNode::addMarker(std::vector<geometry_msgs::Pose> waypoints)
  {
    /*
      https://wiki.ros.org/rviz/DisplayTypes/Marker#The_Marker_Message
    */
    std::vector<geometry_msgs::Pose> waypoints_ik;
    marker_array.markers.resize(waypoints.size());

    for(int i=0; i<waypoints.size(); i++)
    {
      marker_array.markers[i].header.frame_id = frame_id_;
      marker_array.markers[i].header.stamp = ros::Time::now();
      // marker_array.markers[i].ns = "points_and_lines";
      marker_array.markers[i].id = i;
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
      // marker_array.markers[i].type = visualization_msgs::Marker::SPHERE;
      marker_array.markers[i].type = visualization_msgs::Marker::ARROW;

      // std::cout << "msg->circles[i].center.x: " << msg->circles[i].center.x << std::endl;
      marker_array.markers[i].pose.position.x = waypoints[i].position.x;
      marker_array.markers[i].pose.position.y = waypoints[i].position.y;
      marker_array.markers[i].pose.position.z = waypoints[i].position.z;
      marker_array.markers[i].pose.orientation.x = waypoints[i].orientation.x;
      marker_array.markers[i].pose.orientation.y = waypoints[i].orientation.y;
      marker_array.markers[i].pose.orientation.z = waypoints[i].orientation.z;
      marker_array.markers[i].pose.orientation.w = waypoints[i].orientation.w;
      // std::cout << "msg->circles[i].radius " << msg->circles[i].radius << std::endl;
      // marker_array.markers[i].scale.x = 0.01;
      // marker_array.markers[i].scale.y = 0.01;
      // marker_array.markers[i].scale.z = 0.01;
      marker_array.markers[i].scale.x = 0.02;
      marker_array.markers[i].scale.y = 0.01;
      marker_array.markers[i].scale.z = 0.01;
      marker_array.markers[i].color.a = 1.0;

      if(checkWayPointValidity(waypoints[i]))
      {
        marker_array.markers[i].color.r = 0;
        marker_array.markers[i].color.g = 1;
        marker_array.markers[i].color.b = 0;  

        waypoints_ik.push_back(waypoints[i]);
      }
      else
      {
        marker_array.markers[i].color.r = 1;
        marker_array.markers[i].color.g = 0;
        marker_array.markers[i].color.b = 0;
      }
    }
    marker_pub.publish(marker_array);
  }

  void QNode::deleteMarker()
  {
    /*
      DELETE ALL MARKERS
    */
    marker_array.markers.resize(1);
    marker_array.markers[0].header.frame_id = frame_id_;
    marker_array.markers[0].header.stamp = ros::Time::now();
    marker_array.markers[0].id = 0;
    marker_array.markers[0].action = visualization_msgs::Marker::DELETEALL;
    marker_pub.publish(marker_array);
  }

  bool QNode::operateGripper(const int op, float value)
  {
    // RobotState contains the current position/velocity/acceleration data
    moveit::core::RobotStatePtr gripper_current_state = gripper_move_group->getCurrentState();

    // get the current set of joint values for the group.
    std::vector<double> gripper_joint_positions;
    gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);

    //ROS_INFO("No. of joints in eef group is %ld",gripper_joint_positions.size());
    
    bool success;
    switch(op)
    {
      case 0:
        ROS_INFO_STREAM("Initiating gripper full close action...");
        break;
      case 1:
        ROS_INFO_STREAM("Initiating gripper full open action...");
        break;
      case 2:
        if(value>gripper_joint_positions[0])
        {
          ROS_INFO_STREAM("Initiating close gripper action to "<< value <<".");
        }
        else if(value<gripper_joint_positions[0])
        {
          ROS_INFO_STREAM("Initiating open gripper action to "<< value <<".");
        }
        else
        {
          ROS_INFO_STREAM("Gripper's desired state is same as current state.");
        }
        break;
    }

    gripper_joint_positions[0] = value;
    gripper_move_group->setJointValueTarget(gripper_joint_positions);
    //ros::Duration(1.0).sleep();
    success = static_cast<bool>(gripper_move_group->move());
    ROS_INFO("Gripper operation is %s",(success==true)? "success":"failure"); 
    return success;
  }
}  // namespace trajectory_planning_gui