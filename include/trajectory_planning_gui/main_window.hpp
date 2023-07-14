#ifndef TRAJECTORY_PLANNING_GUI_MAIN_WINDOW_H
#define TRAJECTORY_PLANNING_GUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <QtGui>
#include <ios>
#include <iostream>
#include <sstream>
#include <cmath>
#include <unistd.h>
#include <time.h>

#include <QTimer>
#include <eigen3/Eigen/Eigen>
#include <string.h>
#include <QTreeView>

#include <QFileDialog>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <QMessageBox>
#include <QProgressBar>
#include <QProgressDialog>

#include <QWidget>
#include <QSettings>

#include <QtMultimedia>
#include <QtMultimediaWidgets>

#include <QString>
#include <QUrl>
#include <QKeySequenceEdit>
#include <QDebug>

#include <QStandardPaths>
#include <QDir>
#include <QDateTime>
#include <QErrorMessage>

#include "../include/trajectory_planning_gui/point_tree_model.hpp"
#include "../include/trajectory_planning_gui/TabProxy.h"

// macros
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

#define HAVE_NEW_YAMLCPP

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace trajectory_planning_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/

class MainWindow : public QMainWindow{
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
  void writeLog(QString str);
  int convertSpinBoxValueToSlider(double value);
  void setSliderStyle(QSlider *qslider);

  QStringList* waypoints_list;
  QStringListModel* waypoints_listModel;
  std::string joint_name_selected;

public Q_SLOTS:
  void timerCallback();

  void on_home(void);
  void on_connect(void);
  void on_start(void);
  void on_settings(void);
  void on_teachRobot(void);
  void on_info(void);
  void on_refresh(void);

  void on_realModeRB(bool checked);
  void on_simModeRB(bool checked);

  // TEACH MODE SECTION
  void on_teachModeOn(void);
  void on_teachModeOff(void);
  void on_addTeachWaypoint(void);
  void add_geometry();
  void on_geometryComboBoxIndexChanged(void);
  void addCurrentStateAsWaypoint(void);

  void errorMessage(const QString &message);
  int showErrorMessage(std::string title, std::string error);
  
  void on_btn_init_pose_clicked(void);
  void on_btn_home_pose_clicked(void);
  void on_btn_random_pose_clicked(void);
  void on_btn_up_pose_clicked(void);

  // GRIPPER CONTROLLER
  void on_btn_set_gripper_clicked(void);
  void on_btn_gripper_open_clicked(void);
  void on_btn_gripper_close_clicked(void);

  // JOINT SPACE
  void read_joint_angles(void);
  void on_btn_read_joint_angle_clicked(void);
  void on_btn_send_joint_angle_clicked(void);

  // CARTESIAN SPACE
  void addAsWaypoint(void);
  void on_btn_read_kinematic_pose_clicked(void);
  void on_btn_send_kinematic_pose_clicked(void);

  void get_current_state(void);
  void on_btnAddWaypoint(void);
  void on_pause();
  void on_btnCancelTrajectory(void);
  void on_btnSaveTrajectory(void);
  void on_btnLoadTrajectory(void);
  void on_btnRunTrajectory(void);
  void on_btnClearTrajectory(void);
  void on_btn_set_clicked(void);
  void on_btn_reset_clicked(void);
  void on_comboBoxIndexChanged(void);
  // void on_secondComboBoxIndexChanged(void);
  // void on_btn_save_trajectory(void);
  void on_jointNamesComboBoxIndexChanged(void);

  // Get GUI Control Mode (state saved during last session)
  void get_saved_state();

  // change reference frame 
  void on_saveReferenceFrame();
  void on_referenceFrameComboBoxIndexChanged(void);

  void on_btnVelScaling(void);
  void save_comm_settings(void);
  void on_env_var_cb_stateChanged(int state);
  void writeSettings();
  void readSettings();
  void writeSettingsConfirmation();

  void tabSelected();
  void tabMainSelected();
  tf::Transform get_point_pos(void);
  void add_point_UI(void);
  void add_point(const tf::Transform& point_pos);
  void insert_row(const tf::Transform& point_pos,const int count);
  void remove_row(void);
  void add_point_before(void);
  void execute_cartesian_trajectory(void);

  void ChangeSpinBox1(int sliderValue);
  void ChangeSlider1(double spinBoxValue);

  void ChangeSpinBox2(int sliderValue);
  void ChangeSlider2(double spinBoxValue);

  void ChangeSpinBox3(int sliderValue);
  void ChangeSlider3(double spinBoxValue);

  void ChangeSpinBox4(int sliderValue);
  void ChangeSlider4(double spinBoxValue);

  void ChangeSpinBox5(int sliderValue);
  void ChangeSlider5(double spinBoxValue);

  void ChangeSpinBox6(int sliderValue);
  void ChangeSlider6(double spinBoxValue);

  void ChangeVelScaleSlider(double spinBoxValue);
  void ChangeVelScaleSpinBox(int sliderValue);

  // Jog Frame Mode parameters
  void jogFramepublish();
  void enableJogFrameMode(bool checked);
  void respondPositionSliderChanged(int value);
  void respondPositionSliderReleased();
  void respondOrientationSliderChanged(int value);
  void respondOrientationSliderReleased();
  void initPositionAxisComboBox();
  void initOrientationAxisComboBox();
  void onPositionAxisChanged(int index);
  void onOrientationAxisChanged(int index);

  // Jog Joint Mode parameters
  void jogJointpublish();
  void enableJogJointMode(bool checked);
  void respondBaseSliderReleased();
  void respondShoulderSliderReleased();
  void respondElbowSliderReleased();
  void respondWrist1SliderReleased();
  void respondWrist2SliderReleased();
  void respondWrist3SliderReleased();

  // Jog Joystick Mode parameters
  void checkJoyConnectedStatus();
  void enableJogJoystickMode(bool checked);
  void jogJoystickJointpublish();
  void on_jointControlRB(bool checked);
  void on_toolControlRB(bool checked);

  void onTrajectoryPlanCancelled();
  void onTrajectoryExecutionFinished();
  void onCartesianPathExecutionStarted();
  void onCartesianPathCompleted(double fraction);
  void onCartesianPathExecutionFinished();

  // Send a signal that a save the waypoints to a file.
  void savePointsToFile();
  // Send a signal that a load the waypoints from a file.
  void loadPointsFromFile();
  void clearWayPoints();

  // add reference frames to referenceFrameCB combo box
  void add_reference_frames(bool sim);

  const char* BoolToString(bool b);
  const std::string currentDateTime();

Q_SIGNALS:
  void currentIndexChanged(int index);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  bool connection_flag = false; // track if ros is connected or not
  QTimer *timer;
  std::vector<double> waypoint_in_angle;
  std::vector<std::vector<double>> waypoints;
  // cartesian based waypoints
  std::vector<tf::Transform> waypoints_pos;
  int waypoint_index;
  std::vector<std::string> imageTopic;
  const std::string joint_names_cb[6] = {"Base Joint", "Shoulder Joint", "Elbow Joint", "Wrist1 Joint", "Wrist2 Joint", "Wrist3 Joint"};
  const std::string joint_names[6] = {"BJ", "SJ", "EJ", "W1J", "W2J", "W3J"};
  double joint_velocities[6] = {3.927, 3.927, 3.927, 4.488, 4.488, 4.488};
  double joint_accelerations[6] = {0.3272, 0.3272, 0.3272, 0.3739, 0.3739, 0.3739};
  // count cartesian waypoints
  int waypoints_count;

  // reference frames
  std::vector<std::string> rframe = {"world", "base_link"};
  std::string rframe_selected;

  std::string frame_id_;
  const std::string target_link_id_ = "fts_toolside";
  const std::string group_id_ = "arm_manipulator";
  std::string pos_axis_id_;
  std::string rot_axis_id_;
  boost::mutex mutex_;

  double pos_jog_value_; // -0.05<=pos_jog_value_<=0.05
  double rot_jog_value_; // -0.05<=rot_jog_value_<=0.05

  // PATH GEOMETRY: TEACH SECTION
  std::vector<std::string> geometry = {"Straight",
                                       "Arc",
                                       "Circle",
                                       "Custom"};

  // true : simulation
  // false: real robot
  bool sim_mode, requested_mode;  
 
  // USED FOR FILE INPUT(READING)
  std::ifstream infile;
      
  // USED FOR FILE OUTPUT(WRITING) IN APPEND MODE (AT THE END OF THE FILE)
  std::ofstream outfile;                                
};

}  // namespace trajectory_planning_gui

#endif // TRAJECTORY_PLANNING_GUI_MAIN_WINDOW_H