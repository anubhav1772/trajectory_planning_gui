/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TRAJECTORY_PLANNING_GUI_QNODE_HPP_
#define TRAJECTORY_PLANNING_GUI_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

// Rviz Visualization Tool
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <exception>
#include <math.h>

// Boost headers
#include <boost/shared_ptr.hpp>
#include "control_msgs/QueryTrajectoryState.h"
#include "ros/service_client.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

#include <cv_bridge/cv_bridge.h>

#include <QtGui>
#include <QLabel>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>

#include <trajectory_planning_gui/JogFrame.h>
#include <trajectory_planning_gui/JogJoint.h>
#include <trajectory_planning_gui/MoveJoint.h>

#include <sensor_msgs/Joy.h>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/display.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace trajectory_planning_gui {

  // action interface type
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
  typedef boost::shared_ptr<arm_control_client> arm_control_client_Ptr;
  typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
  typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> GripperMoveGroupPtr;

  typedef struct
  {
    public:
      std::string action_ns;
      std::string type;
      std::vector<std::string> joints;
  } Controller;

  typedef struct
  {
    public:
      int linear_button;
      int angular_button;
      std::map<std::string, int> axis_linear;
      std::map<std::string, double> scale_linear;
      std::map<std::string, int> axis_angular;
      std::map<std::string, double> scale_angular;
  } Joystick;

  /*****************************************************************************
  ** Class
  *****************************************************************************/
  class QNode : public QThread{
    Q_OBJECT
    public:
      QNode(int argc, char** argv);
      virtual ~QNode();
      bool init(QLabel* qlabel_1);
      bool init(const std::string &master_url, const std::string &host_url, QLabel* qLabel_1);
      void initialize();
      void run();

      /*********************
      ** Logging
      **********************/
      enum LogLevel {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
      };

      QStringListModel* loggingModel() { return &logging_model; }
      void log( const LogLevel &level, const std::string &msg);

      // GRIPPER CONTROLLER
      void init_gripper_controller();
      bool operateGripper(const int op, float value); // op - operation (0-full close, 1-full open, 2-random)

      // Set Robot Control Mode 
      void set_robot_mode(bool sim);
      bool get_robot_mode();

      void updateRobotState();
      void init_joint_control();
      void control_joint_cb(trajectory_planning_gui::MoveJointConstPtr msg);
      void send_joint_data(std::vector<double> joint_angles, double joint_velocities[6], double joint_accelerations[6]);

      void init_robot3D();
      void load_robot3D();
      rviz::RenderPanel* attach_rviz_render_panel();

      void delete_visual_tools();
      void visualize_path_on_load(std::vector<tf::Transform> waypoints_pos);

      double to_degrees(double radians);
      double from_degrees(double degrees);

      // REFERENCE FRAME
      void set_reference_frame(const std::string frame_id);

      std::vector<double> getPresentJointAngle();
      std::vector<double> getPresentKinematicsPosition();

      bool setRandomPose();
      bool setHomePose();
      bool setUpPose();
      bool setInitPose();
      bool setJointSpacePath(std::vector<double> joint_angle);
      bool setTaskSpacePath(std::vector<double> kinematics_pose);
      tf::Quaternion RPYToQuaternion(float R, float P, float Y);
      geometry_msgs::Vector3 QuaternionToRPY(const geometry_msgs::Quaternion msg);

      void createArmClient(arm_control_client_Ptr& actionClient);
      void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal, int iterations, std::vector<std::vector<double>> waypoints, double joint_velocities[], double joint_accelerations[]);
      void run_trajectory(int iterations, std::vector<std::vector<double>> waypoints, double joint_velocities[], double joint_accelerations[]);
      // update initial pose
      void call_query_state_client();
      // cancel a trajectory planning in the middle of execution
      void cancel_trajectory();
      void pause_trajectory();

      // camera data visualization
      void set_camera_id(int id);
      void init_cam_subscriber();
      void change_cam_subscriber(int camera_id);
      void robotCamViewer(const sensor_msgs::ImageConstPtr& msg);
      // void robotCamViewerLower(const sensor_msgs::ImageConstPtr& msg);
      QPixmap rgb_msg_to_pixmap(const sensor_msgs::ImageConstPtr& msg);
      // QPixmap rgb_msg_to_pixmap_lower(const sensor_msgs::ImageConstPtr& msg);

      // cartesian path planning
      // checks if the Way-Point is within the valid IK solution for the Robot
      bool checkWayPointValidity(const geometry_msgs::Pose& waypoint);
      void execute_cartesian_path(std::vector<geometry_msgs::Pose> waypoints);
      geometry_msgs::Pose parseWayPoint(const tf::Transform& point_pos);
      std::vector<geometry_msgs::Pose> parseWayPoints(std::vector<tf::Transform> waypoints_pos);
      void cartesianPathHandler(std::vector<tf::Transform> waypoints_pos);
      void signal_on_trajectory_completion(arm_control_client_Ptr& actionClient);
      void cartesian_curve(std::vector<tf::Transform> waypoints_pos, int index, float angle_resolution=0.0);
      void waypoints_arc(std::vector<geometry_msgs::Pose> &waypoints, const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose, float angle_resolution);
      void waypoints_straight(std::vector<geometry_msgs::Pose> &waypoints, const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose);
      void addMarker(std::vector<geometry_msgs::Pose> waypoints);
      void deleteMarker();

      void get_joint_max_velocities();
      
      // Update max velocity scaling factor
      void update_max_vel_scale_factor(double value);
      int get_max_vel_scale_factor(void);

      void publish_jog_frame_data(const trajectory_planning_gui::JogFrame msg);
      
      void jog_frame_cb (trajectory_planning_gui::JogFrameConstPtr msg);
      std::vector<double> frame_transform(std::string frame_id_, std::string target_link_id_);

      void joint_state_cb (sensor_msgs::JointStateConstPtr msg);
      int get_controller_list();
      // init jog frame & joint mode parameters
      void init_jog_mode();

      void jog_joint_cb(trajectory_planning_gui::JogJointConstPtr msg);
      void publish_jog_joint_data(const trajectory_planning_gui::JogJoint msg);
      void joy_joint_state_cb(const sensor_msgs::Joy::ConstPtr& joy_msg);
      void joy_frame_state_cb(const sensor_msgs::Joy::ConstPtr& joy_msg);
      void init_joy_jog_mode();
      void init_joy_joint_control_mode();
      void init_joy_frame_control_mode();
      void disable_joy_control_mode();

      // TEACH MODE
      bool get_teach_mode_status();
      void send_teach_mode_signal(int flag);
      ros::Publisher teach_pub; 

    Q_SIGNALS:
      void rosShutdown();
      // Let the RQT Widget know that a trajectory execution is cancelled.
      void trajectoryPlanCancelled();
      // Let the RQT Widget know that a trajectory execution is completed.
      void trajectoryExecutionFinished();
      // Let the RQT Widget know that a Cartesian Path Execution has started.
      void cartesianPathExecuteStarted();
      // Let the RQT Widget know that Cartesian Path Execution has finished.
      void cartesianPathExecuteFinished();
      // Send the percantage of successful completion of the Cartesian Path.
      void cartesianPathCompleted(double fraction);

    private:
      int init_argc;
      char** init_argv;
      QStringListModel logging_model;

      bool sim_mode;
      bool teach_mode_flag;

      rviz::RenderPanel *render_panel_;
	    rviz::VisualizationManager *manager_;

      // For visualizing things in rviz
      moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

      std::vector<double> present_joint_angle_;
      std::vector<double> present_kinematics_position_;

      // MoveIt! interface
      // moveit::planning_interface::MoveGroupInterface* arm_move_group;
      MoveGroupPtr arm_move_group;
      arm_control_client_Ptr ArmClient;
      std::vector<double> initial_pos;
      ros::ServiceClient query_state_client;

      const std::string GRIPPER_PLANNING_GROUP = "gripper";
      const std::string ARM_PLANNING_GROUP = "arm_manipulator";
      const std::vector<std::string> joint_names{"BJ", "SJ", "EJ", "W1J", "W2J", "W3J", "finger_joint"};
      std::vector<double> max_vel;   
      const double min_traj_dur = 5.0;
      double max_velocity_scaling_factor;
      double max_acceleration_scaling_factor;

      const robot_state::JointModelGroup *arm_joint_model_group;
      moveit::core::RobotStatePtr kinematic_state;
      tf::Transform end_effector;

      // GRIPPER
      // moveit::planning_interface::MoveGroupInterface gripper_move_group;
      GripperMoveGroupPtr gripper_move_group;
      const robot_state::JointModelGroup *gripper_joint_model_group;

      // Parameter for setting the planning time of the MoveIt.
      double PLAN_TIME_;
      // Parameter for setting the Cartesian Path step size.
      double CART_STEP_SIZE_;
      // Parameter for setting the Jump Threshold of the Cartesian Path.
      double CART_JUMP_THRESH_;
      // Allow MoveIt to replan.
      bool MOVEIT_REPLAN_;
      // Generate Cartesian Path that avoids collisions.
      bool AVOID_COLLISIONS_;

      // Get the User entered MoveIt and Cartesian Path parameters and pass them to the corresponding private variables.
      void setCartParams(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_);

      // Get the Way-Points from the RViz enviroment and use them to generate Cartesian Path.
      void moveToPose(std::vector<geometry_msgs::Pose> waypoints);

      // Thrid create a IterativeParabolicTimeParameterization object
      trajectory_processing::IterativeParabolicTimeParameterization iptp;

      // camera data visualization parameters
      int camera_id;
      ros::Subscriber image_sub;
      // ros::Subscriber image_sub_2;
      QLabel* imageViewLabel_1;
      // QLabel* imageViewLabel_2; 

      std::vector<std::string> group_names_;
      std::vector<std::string> link_names_;
      ros::Publisher jog_frame_pub_; 
    
      // Jog Frame Mode parameters
      ros::Subscriber joint_state_sub_, jog_frame_sub_;
      ros::ServiceClient fk_client_, ik_client_;

      std::map<std::string, Controller> cinfo_map_;
      std::map<std::string, arm_control_client*> traj_clients_;
      std::map<std::string,ros::Publisher> traj_pubs_;

      std::map<std::string, double> joint_map_;
      geometry_msgs::PoseStamped pose_stamped_;

      std::string target_link_;
      // REFERENCE FRAME ID
      std::string frame_id_;
      std::string group_name_;
      std::vector<std::string> exclude_joints_;
      sensor_msgs::JointState joint_state_;

      std::vector<std::string> joint_name_;
      int jog_joint_num_;
      std::vector<double> jog_value_;
      ros::Publisher jog_joint_pub_;
      ros::Subscriber jog_joint_sub_;

      ros::Publisher joint_state_pub_;
        
      ros::Time last_stamp_;
      double time_from_start_;
      bool use_action_;
      bool intermittent_;
      
      // USED TO STORE THE JOINT VALUES WHEN CANCEL TRAJECTORY IS PRESSED
      std::vector<double> stop_pos;

      ros::Subscriber joy_joint_sub;
      ros::Subscriber joy_frame_sub;
      const std::string joy_name = "ps4";
      std::map<std::string, Joystick> joy_map_;
      // int linear_button, angular_button;
      // std::map<std::string, int> axis_linear; 

      bool joy_mode_flag; 

      ros::Publisher move_joint_pub_;
      ros::Subscriber move_joint_sub_;

      ros::Publisher marker_pub;
      visualization_msgs::MarkerArray marker_array;
  };
}  // namespace trajectory_planning_gui

#endif /* TRAJECTORY_PLANNING_GUI_QNODE_HPP_ */