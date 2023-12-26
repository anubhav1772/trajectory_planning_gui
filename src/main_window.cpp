#include "../include/trajectory_planning_gui/main_window.hpp"

namespace trajectory_planning_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent), 
      qnode(argc,argv)
  {
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    // this->setWindowFlags(Qt::FramelessWindowHint); //Makes the frame invisible
    // this->showMaximized();

    // qApp is a global variable for the application
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); 

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    connect(ui.home, SIGNAL(clicked()), this, SLOT(on_home()));
    connect(ui.refresh, SIGNAL(clicked()), this, SLOT(on_refresh()));
    connect(ui.connect, SIGNAL(clicked()), this, SLOT(on_connect()));
    connect(ui.teachRobot, SIGNAL(clicked()), this, SLOT(on_teachRobot()));
    connect(ui.settings, SIGNAL(clicked()), this, SLOT(on_settings()));
    connect(ui.info, SIGNAL(clicked()), this, SLOT(on_info()));

    connect(ui.tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabSelected()));
    connect(ui.tab_widget_main, SIGNAL(currentChanged(int)), this, SLOT(tabMainSelected()));

    ui.tab_widget_main->setTabPosition(QTabWidget::West);
    ui.tab_widget_main->tabBar()->setStyle(new HorizontalTabStyle);

    connect(ui.simModeRB, SIGNAL(toggled(bool)), this, SLOT(on_simModeRB(bool)));
    connect(ui.realModeRB, SIGNAL(toggled(bool)), this, SLOT(on_realModeRB(bool)));

    connect(ui.horizontalSlider_1, SIGNAL(valueChanged(int)), SLOT(ChangeSpinBox1(int)));
    connect(ui.doubleSpinBox_j1, SIGNAL(valueChanged(double)), SLOT(ChangeSlider1(double)));

    connect(ui.horizontalSlider_2, SIGNAL(valueChanged(int)), SLOT(ChangeSpinBox2(int)));
    connect(ui.doubleSpinBox_j2, SIGNAL(valueChanged(double)), SLOT(ChangeSlider2(double)));

    connect(ui.horizontalSlider_3, SIGNAL(valueChanged(int)), SLOT(ChangeSpinBox3(int)));
    connect(ui.doubleSpinBox_j3, SIGNAL(valueChanged(double)), SLOT(ChangeSlider3(double)));

    connect(ui.horizontalSlider_4, SIGNAL(valueChanged(int)), SLOT(ChangeSpinBox4(int)));
    connect(ui.doubleSpinBox_j4, SIGNAL(valueChanged(double)), SLOT(ChangeSlider4(double)));

    connect(ui.horizontalSlider_5, SIGNAL(valueChanged(int)), SLOT(ChangeSpinBox5(int)));
    connect(ui.doubleSpinBox_j5, SIGNAL(valueChanged(double)), SLOT(ChangeSlider5(double)));

    connect(ui.horizontalSlider_6, SIGNAL(valueChanged(int)), SLOT(ChangeSpinBox6(int)));
    connect(ui.doubleSpinBox_j6, SIGNAL(valueChanged(double)), SLOT(ChangeSlider6(double)));

    connect(ui.vel_scale_factor_slider, SIGNAL(valueChanged(int)), SLOT(ChangeVelScaleSpinBox(int)));
    connect(ui.vel_scale_dsb, SIGNAL(valueChanged(double)), SLOT(ChangeVelScaleSlider(double)));
    connect(ui.btn_set_vel_scale_factor, SIGNAL(clicked()), this,SLOT(on_btnVelScaling()));

    connect(ui.btn_save_comm_settings, SIGNAL(clicked()), this, SLOT(save_comm_settings()));
    connect(ui.saveReferenceFrame, SIGNAL(clicked()), this, SLOT(on_saveReferenceFrame()));

    // PATH PLANNING(JOINT SPACE)
    connect(ui.btn_add_waypoint, &QPushButton::released, this, &MainWindow::on_btnAddWaypoint);
    connect(ui.pause, &QPushButton::released, this, &MainWindow::on_pause);
    connect(ui.btn_cancel_trajectory, &QPushButton::released, this, &MainWindow::on_btnCancelTrajectory);
    connect(ui.btn_run_trajectory, &QPushButton::released, this, &MainWindow::on_btnRunTrajectory);
    connect(ui.btn_clear_trajectory, &QPushButton::released, this, &MainWindow::on_btnClearTrajectory);
    connect(ui.btn_save_trajectory, &QPushButton::released, this, &MainWindow::on_btnSaveTrajectory);
    connect(ui.btn_load_trajectory, &QPushButton::released, this, &MainWindow::on_btnLoadTrajectory);

    // PATH PLANNING(CARTESIAN SPACE/TASK SPACE)
    connect(ui.btn_add_point,SIGNAL(clicked()),this,SLOT(add_point_UI()));
    connect(ui.btn_remove_point,SIGNAL(clicked()),this,SLOT(remove_row()));
    connect(ui.btn_add_point_before,SIGNAL(clicked()),this,SLOT(add_point_before()));
    connect(ui.btn_execute_trajectory,SIGNAL(clicked()),this,SLOT(execute_cartesian_trajectory()));
    connect(ui.btn_save_waypoints, SIGNAL(clicked()), this, SLOT(savePointsToFile()));
    connect(ui.btn_load_waypoints, SIGNAL(clicked()), this, SLOT(loadPointsFromFile()));
    connect(ui.btn_clear_waypoints, SIGNAL(clicked()), this, SLOT(clearWayPoints()));

    // CARTESIAN SPACE
    connect(ui.btn_add_waypoint_ts, SIGNAL(clicked()), this, SLOT(addAsWaypoint()));

    connect(&qnode, SIGNAL(trajectoryPlanCancelled()), this, SLOT(onTrajectoryPlanCancelled()));
    connect(&qnode, SIGNAL(trajectoryExecutionFinished()), this, SLOT(onTrajectoryExecutionFinished()));
    connect(&qnode, SIGNAL(cartesianPathExecuteStarted()), this, SLOT(onCartesianPathExecutionStarted()));
    connect(&qnode, SIGNAL(cartesianPathCompleted(double)), this, SLOT(onCartesianPathCompleted(double)));
    connect(&qnode, SIGNAL(cartesianPathExecuteFinished()), this, SLOT(onCartesianPathExecutionFinished()));

    // Jog Frame Mode section
    connect(ui.jog_frame_enable_btn, SIGNAL(toggled(bool)), this, SLOT(enableJogFrameMode(bool)));
    connect(ui.pos_jog_hs, SIGNAL(valueChanged(int)), this, SLOT(respondPositionSliderChanged(int)));
    connect(ui.pos_jog_hs, SIGNAL(sliderReleased()), this, SLOT(respondPositionSliderReleased()));
    connect(ui.rot_jog_hs, SIGNAL(valueChanged(int)), this, SLOT(respondOrientationSliderChanged(int)));
    connect(ui.rot_jog_hs, SIGNAL(sliderReleased()), this, SLOT(respondOrientationSliderReleased()));
    connect(ui.pos_axis_cb, SIGNAL(activated(int)), this, SLOT(onPositionAxisChanged(int)));
    connect(ui.rot_axis_cb, SIGNAL(activated(int)), this, SLOT(onOrientationAxisChanged(int)));

    // Jog Joint Mode section
    connect(ui.jog_joint_enable_btn, SIGNAL(toggled(bool)), this, SLOT(enableJogJointMode(bool)));
    connect(ui.base_jog_vs, SIGNAL(sliderReleased()), this, SLOT(respondBaseSliderReleased()));
    connect(ui.shoulder_jog_vs, SIGNAL(sliderReleased()), this, SLOT(respondShoulderSliderReleased()));
    connect(ui.elbow_jog_vs, SIGNAL(sliderReleased()), this, SLOT(respondElbowSliderReleased()));
    connect(ui.wrist1_jog_vs, SIGNAL(sliderReleased()), this, SLOT(respondWrist1SliderReleased()));
    connect(ui.wrist2_jog_vs, SIGNAL(sliderReleased()), this, SLOT(respondWrist2SliderReleased()));
    connect(ui.wrist3_jog_vs, SIGNAL(sliderReleased()), this, SLOT(respondWrist3SliderReleased()));

    // Jog Joystick Mode section
    connect(ui.checkJoyConnectedBtn, SIGNAL(clicked()), this, SLOT(checkJoyConnectedStatus()));
    connect(ui.jog_joystick_enable_btn, SIGNAL(toggled(bool)), this, SLOT(enableJogJoystickMode(bool)));
    connect(ui.jointControlRB, SIGNAL(toggled(bool)), this, SLOT(on_jointControlRB(bool)));
    connect(ui.toolControlRB, SIGNAL(toggled(bool)), this, SLOT(on_toolControlRB(bool)));

    // TEACH SECTION
    connect(ui.teachModeOn, SIGNAL(clicked()), this, SLOT(on_teachModeOn()));
    connect(ui.teachModeOff, SIGNAL(clicked()), this, SLOT(on_teachModeOff()));
    connect(ui.addTeachWaypoint, SIGNAL(clicked()), this, SLOT(on_addTeachWaypoint()));
    connect(ui.clearTeachWaypoints, SIGNAL(clicked()), this, SLOT(clearWayPoints()));

    ui.refresh->setToolTip(tr("Reload with changed settings"));

    ui.btn_save_waypoints->setToolTip(tr("Save Way-Points to a file"));
    ui.btn_load_waypoints->setToolTip(tr("Load Way-Points from a file"));
    ui.btn_clear_waypoints->setToolTip(tr("Clear all Way-Points"));
    ui.btn_add_point->setToolTip(tr("Add a new Way-Point"));
    ui.btn_remove_point->setToolTip(tr("Remove a selected Way-Point"));

    imageTopic.push_back("/camera/rgb/image_raw");
    // imageTopic.push_back("/camera/depth_registered/image_raw");

    ui.joint_names_comboBox->addItem(QString::fromStdString("<null>"));

    waypoints_list = new QStringList();
    waypoints_listModel = new QStringListModel(this);
    waypoints_listModel->setStringList(*waypoints_list);

    ui.waypointslistView->setModel(waypoints_listModel);
    ui.waypointslistView->setEditTriggers(QAbstractItemView::NoEditTriggers);

    waypoint_index = 0;
    waypoints_count = 0;

    QStringList headers;
    headers<<tr("Point")<<tr("Position (m)")<<tr("Orientation (deg)");
    PointTreeModel *model = new PointTreeModel(headers,QString::fromStdString("add_point_button"));
    ui.waypoints_treeView->setModel(model);

    setSliderStyle(ui.vel_scale_factor_slider);
    setSliderStyle(ui.horizontalSlider_1);
    setSliderStyle(ui.horizontalSlider_2);
    setSliderStyle(ui.horizontalSlider_3);
    setSliderStyle(ui.horizontalSlider_4);
    setSliderStyle(ui.horizontalSlider_5);
    setSliderStyle(ui.horizontalSlider_6);

    // position horizontal slider parameter settings
    ui.pos_jog_hs->setTickPosition(QSlider::TicksBelow);
    ui.pos_jog_hs->setTickInterval(500);
    ui.pos_jog_hs->setMinimum(-1000);
    ui.pos_jog_hs->setMaximum( 1000);
    ui.pos_jog_hs->setTracking(true);
    ui.pos_jog_hs->setSingleStep(0);
    ui.pos_jog_hs->setPageStep(0);
    setSliderStyle(ui.pos_jog_hs);

    // orientation horizontal slider parameter settings
    ui.rot_jog_hs->setTickPosition(QSlider::TicksBelow);
    ui.rot_jog_hs->setTickInterval(500);
    ui.rot_jog_hs->setMinimum(-1000);
    ui.rot_jog_hs->setMaximum( 1000);
    ui.rot_jog_hs->setTracking(true);
    ui.rot_jog_hs->setSingleStep(0);
    ui.rot_jog_hs->setPageStep(0);
    setSliderStyle(ui.rot_jog_hs);
    
    
    // base joint vertical slider parameter settings
    ui.base_jog_vs->setTickPosition(QSlider::TicksBelow);
    ui.base_jog_vs->setTickInterval(500);
    ui.base_jog_vs->setMinimum(-1000);
    ui.base_jog_vs->setMaximum( 1000);
    ui.base_jog_vs->setTracking(true);
    ui.base_jog_vs->setSingleStep(0);
    ui.base_jog_vs->setPageStep(0);
    setSliderStyle(ui.base_jog_vs);

    // shoulder joint vertical slider parameter settings
    ui.shoulder_jog_vs->setTickPosition(QSlider::TicksBelow);
    ui.shoulder_jog_vs->setTickInterval(500);
    ui.shoulder_jog_vs->setMinimum(-1000);
    ui.shoulder_jog_vs->setMaximum( 1000);
    ui.shoulder_jog_vs->setTracking(true);
    ui.shoulder_jog_vs->setSingleStep(0);
    ui.shoulder_jog_vs->setPageStep(0);
    setSliderStyle(ui.shoulder_jog_vs);
    
    // elbow joint vertical slider parameter settings
    ui.elbow_jog_vs->setTickPosition(QSlider::TicksBelow);
    ui.elbow_jog_vs->setTickInterval(500);
    ui.elbow_jog_vs->setMinimum(-1000);
    ui.elbow_jog_vs->setMaximum( 1000);
    ui.elbow_jog_vs->setTracking(true);
    ui.elbow_jog_vs->setSingleStep(0);
    ui.elbow_jog_vs->setPageStep(0);
    setSliderStyle(ui.elbow_jog_vs);
    
    // wrist1 joint vertical slider parameter settings
    ui.wrist1_jog_vs->setTickPosition(QSlider::TicksBelow);
    ui.wrist1_jog_vs->setTickInterval(500);
    ui.wrist1_jog_vs->setMinimum(-1000);
    ui.wrist1_jog_vs->setMaximum( 1000);
    ui.wrist1_jog_vs->setTracking(true);
    ui.wrist1_jog_vs->setSingleStep(0);
    ui.wrist1_jog_vs->setPageStep(0);
    setSliderStyle(ui.wrist1_jog_vs);
    
    // wrist2 joint vertical slider parameter settings
    ui.wrist2_jog_vs->setTickPosition(QSlider::TicksBelow);
    ui.wrist2_jog_vs->setTickInterval(500);
    ui.wrist2_jog_vs->setMinimum(-1000);
    ui.wrist2_jog_vs->setMaximum( 1000);
    ui.wrist2_jog_vs->setTracking(true);
    ui.wrist2_jog_vs->setSingleStep(0);
    ui.wrist2_jog_vs->setPageStep(0);
    setSliderStyle(ui.wrist2_jog_vs);

    // wrist3 joint vertical slider parameter settings  
    ui.wrist3_jog_vs->setTickPosition(QSlider::TicksBelow);
    ui.wrist3_jog_vs->setTickInterval(500);
    ui.wrist3_jog_vs->setMinimum(-1000);
    ui.wrist3_jog_vs->setMaximum( 1000);
    ui.wrist3_jog_vs->setTracking(true);
    ui.wrist3_jog_vs->setSingleStep(0);
    ui.wrist3_jog_vs->setPageStep(0);
    setSliderStyle(ui.wrist3_jog_vs);
    
    ui.home->setIcon(QIcon(":/images/home.png"));
    ui.home->setIconSize(QSize(32,32));
    ui.home->setStyleSheet("QPushButton{border-radius:5px;border: 1px solid #345781;}");

    ui.bluetooth->setIcon(QIcon(":/images/bluetooth.png"));
    ui.bluetooth->setIconSize(QSize(32,32));
    ui.bluetooth->setStyleSheet("QPushButton{border-radius:5px;border: 1px solid #345781;}");

    // ui.connect->setIcon(QIcon(":/images/power_on.png"));
    QIcon icon = QIcon();
    // 'Off' state corresponds to unchecked state of QPushButton
    icon.addPixmap( QPixmap( ":/images/power_on.png" ), QIcon::Normal, QIcon::Off );
    // 'On' state corresponds to checked state of QPushButton
    icon.addPixmap( QPixmap( ":/images/power_off.png" ), QIcon::Normal, QIcon::On );
    ui.connect->setIcon(icon);
    ui.connect->setIconSize(QSize(32,32));
    ui.connect->setStyleSheet("QPushButton{border-radius:5px;border: 1px solid #345781;}");
    ui.connect->setCheckable(true);
    ui.connect->setChecked(false);

    ui.info->setIcon(QIcon(":/images/info.png"));
    ui.info->setIconSize(QSize(32,32));
    ui.info->setStyleSheet("QPushButton{border-radius:5px;border: 1px solid #345781;}");

    ui.settings->setIcon(QIcon(":/images/settings.png"));
    ui.settings->setIconSize(QSize(32,32));
    ui.settings->setStyleSheet("QPushButton{border-radius:5px;border: 1px solid #345781;}");

    ui.refresh->setIcon(QIcon(":/images/refresh.png"));
    ui.refresh->setIconSize(QSize(32,32));
    ui.refresh->setStyleSheet("QPushButton{border-radius:5px;border: 1px solid #345781;}");

    ui.teachRobot->setIcon(QIcon(":/images/teach.png"));
    ui.teachRobot->setIconSize(QSize(32,32));
    ui.teachRobot->setStyleSheet("QPushButton{border-radius:5px;border: 1px solid #345781;}");

    on_start();
  }

  void MainWindow::on_pause()
  {
    ROS_INFO("Pausing trajectory..."); 
    qnode.pause_trajectory();
    ROS_INFO("Trajectory paused!");
  }

  MainWindow::~MainWindow() 
  {
    outfile.close();
  }


  static QVariant boxValue(const QComboBox* box) 
  {
    int idx = box->currentIndex();
    if (idx == -1)
      return QVariant();

    return box->itemData(idx);
  }

  void MainWindow::timerCallback()
  {
    get_current_state();

    // call publish function only when jog_frame_enable_btn is enabled/checked
    if (ui.jog_frame_enable_btn->isChecked())
    {
      jogFramepublish();
    }
    if (ui.jog_joint_enable_btn->isChecked())
    {
      jogJointpublish();
    }
  }

  /**
    Tasks to be performed on starting the GUI initially
    Not connected with ROS, just the UI part
  **/
  void MainWindow::on_start(void)
  {
    ui.control_parameters_frame->setEnabled(true);
    ui.data_frame->setEnabled(true);
    ui.tabWidget->setEnabled(true);
    ui.upper_frame->setEnabled(true);

    ui.btn_home_pose->setEnabled(true);
    ui.btn_init_pose->setEnabled(true);
    ui.btn_random_pose->setEnabled(true);
    ui.btn_up_pose->setEnabled(true);
    ui.btn_add_waypoint->setEnabled(true);
    ui.btn_read_joint_angle->setEnabled(true);
    ui.btn_read_kinematic_pose->setEnabled(true);
    ui.btn_send_joint_angle->setEnabled(true);
    ui.btn_send_kinematic_pose->setEnabled(true);
    ui.btn_gripper_close->setEnabled(true);
    ui.btn_gripper_open->setEnabled(true);
    ui.btn_set_gripper->setEnabled(true);
    ui.btn_add_waypoint_ts->setEnabled(true);
    ui.waypointslistView->setEnabled(true);

    // enable control parameters
    ui.joint_names_comboBox->setEnabled(true);
    ui.speed_doubleSpinBox->setEnabled(true);
    ui.acc_doubleSpinBox->setEnabled(true);
    ui.btn_set->setEnabled(true);
    ui.btn_reset->setEnabled(true);

    // get all image topics for visualization purpose here
    ui.imageTopicsComboBox->setEnabled(true);
    for(int i=0;i<imageTopic.size();i++)
    {
      ui.imageTopicsComboBox->addItem(QString::fromStdString(imageTopic[i]));
    }
    connect(ui.imageTopicsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(on_comboBoxIndexChanged()));

    ui.joint_names_comboBox->clear();
    for(int i=0;i<6;i++)
    {
      ui.joint_names_comboBox->addItem(QString::fromStdString(joint_names_cb[i]));
    }
    connect(ui.joint_names_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(on_jointNamesComboBoxIndexChanged()));
    
    ui.vel_scale_factor_slider->setValue(qnode.get_max_vel_scale_factor());

    initPositionAxisComboBox();
    initOrientationAxisComboBox();
    
    ui.jog_frame_enable_btn->setStyleSheet("QPushButton:checked { background-color: red; }\n");
    ui.jog_frame_enable_btn->setCheckable(true);
    ui.jog_frame_enable_btn->setChecked(false);

    ui.jog_joint_enable_btn->setStyleSheet("QPushButton:checked { background-color: red; }\n");
    ui.jog_joint_enable_btn->setCheckable(true);
    ui.jog_joint_enable_btn->setChecked(false);

    ui.jog_joystick_enable_btn->setStyleSheet("QPushButton:checked { background-color: red; }\n");
    ui.jog_joystick_enable_btn->setCheckable(true);
    ui.jog_joystick_enable_btn->setChecked(false);
    // controlGroupBox settings(Joystick)
    ui.jointControlRB->setChecked(true);

    // TEACH MODE SECTION
    ui.teachModeOff->setEnabled(false);
    ui.geometryCB->setEnabled(false);
    add_geometry();
    ui.angleResDSB->setEnabled(false);
    connect(ui.geometryCB, SIGNAL(currentIndexChanged(int)), this, SLOT(on_geometryComboBoxIndexChanged())); 

    // Call QNode::get_robot_mode() to get mode (simulation/real robot)
    // sim_mode = true  : simulation
    // sim_mode = false : real robot 
    
    // read last saved state/mode from __mode.txt file and set radio button (simulation/real robot) accordingly
    get_saved_state();
    if(qnode.get_robot_mode())
    {
      ui.simModeRB->setAutoExclusive(false);
      ui.simModeRB->setChecked(true);
      ui.simModeRB->setAutoExclusive(true);

      // NO NEED FOR BELOW CODE AS THIS IS ALREADY BEING DONE BY on_simModeRB() call
      // ui.realModeRB->setAutoExclusive(false);
      // ui.realModeRB->setChecked(false);
      // ui.realModeRB->setAutoExclusive(true);
    }
    else
    {
      // NO NEED FOR BELOW CODE AS THIS IS ALREADY BEING DONE BY on_realModeRB() call
      // ui.simModeRB->setAutoExclusive(false);
      // ui.simModeRB->setChecked(false);
      // ui.simModeRB->setAutoExclusive(true);
      ui.realModeRB->setAutoExclusive(false);
      ui.realModeRB->setChecked(true);
      ui.realModeRB->setAutoExclusive(true);
    }

    // SET REFERENCE FRAME SECTION (UNDER SETTINGS TAB)
    connect(ui.referenceFrameCB, SIGNAL(currentIndexChanged(int)), this, SLOT(on_referenceFrameComboBoxIndexChanged())); 

    int status = mkdir((boost::filesystem::current_path().generic_string() + "/rosdep/meta.cache/log/").c_str(), 0777);
    
    // LOG DATA FILE PATH FULL
    const std::string DATA_FILE_PATH = boost::filesystem::current_path().generic_string() + "/rosdep/meta.cache/log/__data.log";
    
    if((status<0) && (EEXIST !=0))
    {
      infile.open(DATA_FILE_PATH);
      outfile.open(DATA_FILE_PATH, std::ios::app);
      if(infile.fail())
      {
        ROS_ERROR_STREAM("NO DATA FILE EXISTS AT "<<DATA_FILE_PATH);
        writeLog("FILE CREATED SUCCESSFULLY.");
      }
      else
      {
        writeLog("FILE ALREADY EXISTS.");
      }
      infile.close();
    }
    else
    {
      // FOLDER DOESNOT EXIST, CREATING ONE
    }
  }
  
  /**
    read the __mode.txt file to check the last saved state (simulation mode or real robot mode)
    If no such file exists, create a new file with mode as simulation (sim_mode = True)
    Else read the file and save it to sim_mode
  **/
  void MainWindow::get_saved_state()
  {
    // status == 0 (success) else (fail)
    std::string path = boost::filesystem::current_path().generic_string() + "/rosdep/meta.cache/__mode.txt";
    std::ifstream file(path);

    if(file.fail())
    {
      ROS_ERROR_STREAM("NO SUCH FILE EXISTS AT "<<path);
      std::ofstream outfile (path); 
      outfile << "true" << std::endl;

      qnode.set_robot_mode(true);
    }
    else
    {
      std::string content;
      bool mode;
      while(file.is_open() && !file.eof())
      {
        file>>content;
      }
      // For string to boolean conversion
      std::istringstream(content) >> std::boolalpha >> mode;
      qnode.set_robot_mode(mode);
      file.close();
    }
  }

  void MainWindow::add_geometry()
  {
    for(int i=0; i<geometry.size(); i++)
    {
      ui.geometryCB->addItem(QString::fromStdString(geometry[i]));
    }
  }

  void MainWindow::on_geometryComboBoxIndexChanged()
  {
    if(qnode.get_teach_mode_status()) // true, teach mode is ON, hence teachModeOn button is disabled
    {
      int index = ui.geometryCB->currentIndex();
      ROS_INFO_STREAM("Teach Mode on with selected geometry "<<geometry[index]<<".");
      ui.teachModeLog->setStyleSheet("QLabel { color : green; }");
      if(index==0)
      {
        ui.angleResDSB->setEnabled(false);
        ui.teachModeLog->setText(QString::fromStdString("Teach mode enabled with selected geometry "+geometry[index]+". Please add 2 points in 3d space."));
      }
      else if(index==1)
      {
        ui.angleResDSB->setEnabled(true);
        ui.teachModeLog->setText(QString::fromStdString("Teach mode enabled with selected geometry "+geometry[index]+". Please add 2 points in 3d space."));
      }
      else if(index==2)
      {
        ui.angleResDSB->setEnabled(true);
        ui.teachModeLog->setText(QString::fromStdString("Teach mode enabled with selected geometry "+geometry[index]+". Please add 2 points in 3d space."));
      }
      else if(index==3)
      {
        ui.angleResDSB->setEnabled(true);
        ui.teachModeLog->setText(QString::fromStdString("Teach mode enabled with selected geometry "+geometry[index]+"."));
      }
      else
      {
        ui.angleResDSB->setEnabled(false);
        ui.teachModeLog->setText("Teach mode enabled with no selected geometry. Try to select a geometry.");
      }
    }
    else  // false, teach mode is OFF, hence teachModeOn button is enabled and teachModeOff is disabled
    {
      ui.geometryCB->setEnabled(false);
      ui.teachModeLog->setText("Teach mode is disabled. Please enable it to set geometry.");
    } 
  }

  void MainWindow::add_reference_frames(bool sim)
  {
    /**
    if(ui.referenceFrameCB->currentIndex() != -1)
    {
      // By default, for an empty combo box or a combo box in which no current item is set, 
      // this property has a value of -1.
      ui.referenceFrameCB->setCurrentIndex(-1);
      ui.referenceFrameCB->clear();
    }**/

    QSettings settings("robot_settings", "trajectory_planning_gui");

    // True  - simulation mode (only world as reference frame)
    // False - real robot mode ((only base_link as reference frame))
    if(sim)
    {
      QString saved_ref_frame = settings.value("reference_frame", QString("world")).toString();

      // add all reference frames to referenceFrameCB combo box
      for(int i=0; i<rframe.size()-1; i++)
      {
        ui.referenceFrameCB->addItem(QString::fromStdString(rframe[i]));
      }
      ui.referenceFrameCB->setCurrentText(saved_ref_frame.toUtf8().constData());
      qnode.set_reference_frame(saved_ref_frame.toUtf8().constData());

      frame_id_ = "world";
    }
    else
    { 
      QString saved_ref_frame = settings.value("reference_frame", QString("base_link")).toString();

      // add all reference frames except world  
      for(int i=1; i<rframe.size(); i++)
      {
        ui.referenceFrameCB->addItem(QString::fromStdString(rframe[i]));
      }

      if(saved_ref_frame.toUtf8().constData() != rframe[0])
      {
        ui.referenceFrameCB->setCurrentText(saved_ref_frame.toUtf8().constData());
        qnode.set_reference_frame(saved_ref_frame.toUtf8().constData());
      }
      else
      {
        ui.referenceFrameCB->setCurrentText(QString::fromStdString(rframe[1]));
        qnode.set_reference_frame(rframe[1]);
      } 

      frame_id_ = "base_link";
    }
    ROS_INFO_STREAM("reference frame"<<frame_id_);
  }

  void MainWindow::on_teachModeOn(void)
  {
    qnode.send_teach_mode_signal(1);
    ui.teachModeOn->setEnabled(false);
    ui.teachModeOff->setEnabled(true);

    ui.geometryCB->setEnabled(true);

    int index = ui.geometryCB->currentIndex();
    ui.teachModeLog->setStyleSheet("QLabel { color : green; }");
    ui.teachModeLog->setText(QString::fromStdString("Teach mode is enabled with default geometry "+geometry[index]+". Please add 2 points in 3d space."));

    // UNDER CARTESIAN BASED TRAJECTORY PLANNING SECTION
    ui.teachModeCB->setChecked(true);
  }

  void MainWindow::on_teachModeOff(void)
  {
    qnode.send_teach_mode_signal(0);
    ui.teachModeOn->setEnabled(true);
    ui.teachModeOff->setEnabled(false);

    ui.geometryCB->setEnabled(false);
    ui.angleResDSB->setEnabled(false);
    ui.teachModeLog->setText("Teach mode is disabled.");
    ui.teachModeLog->setStyleSheet("QLabel { color : green; }");

    // UNDER CARTESIAN BASED TRAJECTORY PLANNING SECTION
    ui.teachModeCB->setChecked(false);
  }

  void MainWindow::on_addTeachWaypoint(void)
  {
    if(ui.geometryCB->currentIndex()==0)
    {
      /*
        STRAIGHT LINE TRAJECTORY: ONLY 2 POINTS NEED TO BE ADDED
      */
      if(waypoints_pos.size()<2)
      {
        addCurrentStateAsWaypoint();
      }
      else
      {
        errorMessage("Can't add more than 2 points for straight trajectory.");
      }
    }
    else if(ui.geometryCB->currentIndex()==1)
    {
      /*
        ARC SHAPED TRAJECTORY: ONLY 2 POINTS NEED TO BE ADDED
      */
      if(waypoints_pos.size()<2)
      {
        addCurrentStateAsWaypoint();
      }
      else
      {
        errorMessage("Can't add more than 2 points for arc shaped trajectory.");
      }
    }
  }

  void MainWindow::save_comm_settings(void)
  {
    writeSettingsConfirmation();
  }

  void MainWindow::writeSettingsConfirmation()
  {
    QMessageBox msgBox;
    msgBox.setWindowTitle("Settings Modified");
    msgBox.setText("Requested change in the communication settings.");
    msgBox.setInformativeText("Do you want to save your changes?");
    msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    int ret = msgBox.exec();
    
    switch(ret){
      case QMessageBox::Save:
        // save button is clicked
        writeSettings();
        writeLog("Previous communication settings successfully overriden.");
        break;
      case QMessageBox::Cancel:
        // cancel button is clicked
        writeLog("Cancelled request to save new communication settings.");
        break;
      default:
        break;
    }
  }

  void MainWindow::errorMessage(const QString &message)
  {
    QErrorMessage err(this);
    err.showMessage(message);
    err.exec();
  }

  int MainWindow::showErrorMessage(std::string title, std::string error) 
  {
    QMessageBox msgBox;
    msgBox.setWindowTitle(QString::fromStdString(title));
    msgBox.setIconPixmap(QPixmap(":/images/robot_not_connected.png"));
    msgBox.setText(QString::fromStdString(error));
    int ret = msgBox.exec();
    return ret;
  }

  void MainWindow::on_env_var_cb_stateChanged(int state) 
  {
    bool enabled;
    if (state == 0) 
    {
      enabled = true;
    } 
    else 
    {
      enabled = false;
    }
    ui.master_ip->setEnabled(enabled);
    ui.host_ip->setEnabled(enabled);
  }

  /**
    save a particular reference frame in the settings section
    Tasks to be performed:
    1. save a given link as reference frame for future sessions(automatically selected)
    2. set the new reference frame for future calculation during current session
  **/
  void MainWindow::on_saveReferenceFrame()
  {
    rframe_selected = rframe[ui.referenceFrameCB->currentIndex()];

    QSettings settings("robot_settings", "trajectory_planning_gui");
    settings.setValue("reference_frame", QString::fromStdString(rframe_selected));

    ROS_INFO("Saving selected reference frame...");
    ROS_INFO_STREAM("Reference frame selected: " << rframe_selected);
    writeLog(QString::fromStdString(rframe_selected +" is saved as reference frame."));
  }

  void MainWindow::readSettings() 
  {
    writeLog("Reading settings...");
    QSettings settings("robot_settings", "trajectory_planning_gui");

    // SET SAVED REFERENCE FRAME
    // QString saved_ref_frame = settings.value("reference_frame", QString("base_link")).toString();
    
    // ROS_INFO_STREAM("SIMULATION MODE: " << qnode.get_robot_mode());
    // ROS_INFO_STREAM("REFERENCE FRAME: " << saved_ref_frame.toUtf8().constData());

    // True  - simulation mode
    // False - real robot mode
    /**
    if(!qnode.get_robot_mode())
    {
      if(saved_ref_frame.toUtf8().constData() != rframe[0])
      {
        ui.referenceFrameCB->setCurrentText(saved_ref_frame.toUtf8().constData());
      }
      else
      {
        ui.referenceFrameCB->setCurrentText(QString::fromStdString(rframe[1]));
      }  
    }
    else
    {
      ui.referenceFrameCB->setCurrentText(saved_ref_frame.toUtf8().constData());
    }
    **/

    // int index = ui.referenceFrameCB->findData(saved_ref_frame.toUtf8().constData());
    // writeLog(QString::fromStdString("Saved reference frame ")+saved_ref_frame);

    // ROS_INFO_STREAM("index "<<index<<".");
    // if ( index != -1 ) // -1 for not found
    // { 
    //   ROS_INFO("Saved reference frame found. Setting...");
    //   // Reference frame found
    //   ui.referenceFrameCB->setCurrentIndex(index);
    //   qnode.set_reference_frame(saved_ref_frame.toUtf8().constData());
    // }
    // else
    // {
    //   ROS_INFO("Saved reference frame not found.");
    //   // Reference frame not found
    //   ui.referenceFrameCB->setCurrentIndex(0);
    //   qnode.set_reference_frame(rframe[0]);
    // }
    
    QString master_ip = settings.value("master_ip",QString("http://localhost:11311/")).toString();
    QString host_ip = settings.value("host_ip", QString("localhost")).toString();
    ui.master_ip->setText(master_ip);
    ui.host_ip->setText(host_ip);

    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.env_var_cb->setChecked(checked);
    if (checked) 
    {
    	ui.master_ip->setEnabled(false);
    	ui.host_ip->setEnabled(false);
    }
  }

  void MainWindow::writeSettings() 
  {
    writeLog("Writing settings...\n");
    QSettings settings("robot_settings", "trajectory_planning_gui");

    settings.setValue("master_ip", ui.master_ip->text());
    settings.setValue("host_ip", ui.host_ip->text());
    settings.setValue("use_environment_variables",QVariant(ui.env_var_cb->isChecked()));
  }

  void MainWindow::on_referenceFrameComboBoxIndexChanged(void)
  {
    rframe_selected = rframe[ui.referenceFrameCB->currentIndex()];
    writeLog(QString::fromStdString("Selected "+rframe_selected+" ."));

    qnode.set_reference_frame(rframe_selected);
  }

  void MainWindow::on_connect(void)
  {
    if(ui.connect->isChecked())
    {
      ROS_INFO("Starting Pendent Application...");
      readSettings();
      if (!ui.env_var_cb->isChecked()) 
      {
        if (!qnode.init(ui.imageView)) 
        {
          showErrorMessage("Connection Error", "Unable to connect to the robot device. Please check the settings for further details.");
          ui.connect->setChecked(false);
        } 
        else 
        {
          // ui.connect->setEnabled(false);

          timer = new QTimer(this);
          connect(timer, SIGNAL(timeout()), this, SLOT(timerCallback()));
          timer->start(100);

          connection_flag = true;
          qnode.load_robot3D();

          writeLog("QTimer start : 100ms");
        }
      } 
      else 
      {
        // if (!qnode.init(ui.master_ip->text().toStdString(), ui.host_ip->text().toStdString(), ui.imageView, ui.speech2TextCommand)) 
        if (!qnode.init(ui.master_ip->text().toStdString(),
                        ui.host_ip->text().toStdString(),
                        ui.imageView)) 
        {
          showErrorMessage("Connection Error", "Unable to connect to the robot device. Please check the settings for further details.");
          ui.connect->setChecked(false);
        } 
        else 
        {
          // ui.connect->setEnabled(false);
          ui.master_ip->setReadOnly(true);
          ui.host_ip->setReadOnly(true);

          timer = new QTimer(this);
          connect(timer, SIGNAL(timeout()), this, SLOT(timerCallback()));
          timer->start(100);

          connection_flag = true;
          qnode.load_robot3D();

          writeLog("QTimer start : 100ms");
        }
      }
    }
    else
    {
      ROS_INFO("Closing Pendent Application...");
      // close();
      // kill the roslaunch from the system level. 
      // This will close all the nodes.
      system("pkill roslaunch");
    }
  }


  void MainWindow::checkJoyConnectedStatus()
  {
    int status = system("ls /dev/input/ >input_devices.txt"); // execute the linux command "ls /dev/input/"
  
    // status == 0 (success) else (fail)
    std::ifstream file("input_devices.txt");
    std::string device_name;
    bool flag = false;

    while(file.is_open() && !file.eof())
      {
        file>>device_name;
        if(device_name == "js0" || device_name == "js1")
        {
          flag = true;
          break;
        }
      }
    
    file.close();
    
    if(flag)
    {
      ROS_INFO_STREAM("Found input device "<<device_name<<".");
      ui.status_label->setText("Joystick device found.");
      ui.status_label->setStyleSheet("QLabel { color : green; }");
      // ROS_INFO_STREAM("Configuring the "<<device_name<<" device for use.");

      // http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
      // system(("chmod a+rw /dev/input/"+device_name).c_str());
      /**
        sudo: no tty present and no askpass program specified
        https://stackoverflow.com/questions/21659637/how-to-fix-sudo-no-tty-present-and-no-askpass-program-specified-error/24648413#24648413
      **/
    }
    else
    {
      ROS_ERROR("No joystick device found. Check if the device is connected.");
      ui.status_label->setText("No joystick device found. Check if the device is connected.");
      ui.status_label->setStyleSheet("QLabel { color : red; }");
    }   
  }

  void MainWindow::jogJoystickJointpublish()
  {
    qnode.init_joy_jog_mode();
  }

  void MainWindow::jogFramepublish()
  {
    boost::mutex::scoped_lock lock(mutex_);
    
    // publish frame data
    trajectory_planning_gui::JogFrame msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.group_name = group_id_;
    msg.link_name = target_link_id_;
    
    // Position jogging
    if (pos_axis_id_ == "x")
    {
      msg.linear_delta.x = pos_jog_value_;
    }
    if (pos_axis_id_ == "y")
    {
      msg.linear_delta.y = pos_jog_value_;
    }
    if (pos_axis_id_ == "z")
    {
      msg.linear_delta.z = pos_jog_value_;
    }
    // Orientation jogging
    if (rot_axis_id_ == "x")
    {
      msg.angular_delta.x = rot_jog_value_;
    }
    if (rot_axis_id_ == "y")
    {
      msg.angular_delta.y = rot_jog_value_;
    }
    if (rot_axis_id_ == "z")
    {
      msg.angular_delta.z = rot_jog_value_;
    }

    // Publish only if the all command are not equal zero
    // Not good, we need to compare slider value by some way...
    if (msg.linear_delta.x != 0 || msg.linear_delta.y != 0 || msg.linear_delta.z != 0 ||
        msg.angular_delta.x != 0 || msg.angular_delta.y != 0 || msg.angular_delta.z != 0)
    {
      writeLog("publishing joint frame data...");
      qnode.publish_jog_frame_data(msg);

      std::vector<double> transformed_data = qnode.frame_transform(frame_id_, target_link_id_);
      ui.jog_x->setText(QString::number(transformed_data[0], 'f', 3));
      ui.jog_y->setText(QString::number(transformed_data[1], 'f', 3));
      ui.jog_z->setText(QString::number(transformed_data[2], 'f', 3));
      ui.jog_roll->setText(QString::number(transformed_data[3], 'f', 3));
      ui.jog_pitch->setText(QString::number(transformed_data[4], 'f', 3));
      ui.jog_yaw->setText(QString::number(transformed_data[5], 'f', 3));

      // writeLog(QString::fromStdString("x "+std::to_string(transformed_data[0])+"\ny "+std::to_string(transformed_data[1])+"\nz "+std::to_string(transformed_data[2])+"\nroll "+std::to_string(transformed_data[3])+"\npitch "+std::to_string(transformed_data[4])+"\nyaw "+std::to_string(transformed_data[5])));
      // writeLog(QString::fromStdString("*********************************"));
    }
  }  

  void MainWindow::jogJointpublish()
  {
    // publish joint data
    trajectory_planning_gui::JogJoint msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;

    msg.joint_names.resize(6);
    for (int i=0; i<6; i++)
    {
      msg.joint_names[i] = joint_names[i];
    }
      
    msg.deltas.resize(6);
    
    msg.deltas[0] = 0.1 * ui.base_jog_vs->value() / ui.base_jog_vs->maximum();
    msg.deltas[1] = 0.1 * ui.shoulder_jog_vs->value() / ui.shoulder_jog_vs->maximum();
    msg.deltas[2] = 0.1 * ui.elbow_jog_vs->value() / ui.elbow_jog_vs->maximum();
    msg.deltas[3] = 0.1 * ui.wrist1_jog_vs->value() / ui.wrist1_jog_vs->maximum();
    msg.deltas[4] = 0.1 * ui.wrist2_jog_vs->value() / ui.wrist2_jog_vs->maximum();
    msg.deltas[5] = 0.1 * ui.wrist3_jog_vs->value() / ui.wrist3_jog_vs->maximum();

    // Publish only if the all value are not equal zero
    if (ui.base_jog_vs->value() !=0 || ui.shoulder_jog_vs->value() !=0 || 
        ui.elbow_jog_vs->value() !=0 || ui.wrist1_jog_vs->value() !=0 || 
        ui.wrist2_jog_vs->value() !=0 || ui.wrist3_jog_vs->value() !=0)
    {
      writeLog("publishing joint frame data...");
      qnode.publish_jog_joint_data(msg);
    }
  }  

  void MainWindow::onPositionAxisChanged(int index)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pos_axis_id_ = ui.pos_axis_cb->currentText().toStdString();
    ROS_INFO_STREAM("Changed Axis: " << pos_axis_id_);
  }

  void MainWindow::onOrientationAxisChanged(int index)
  {
    boost::mutex::scoped_lock lock(mutex_);
    rot_axis_id_ = ui.rot_axis_cb->currentText().toStdString();
    ROS_INFO_STREAM("Changed Orientation Axis: " << rot_axis_id_);
  }

  void MainWindow::initPositionAxisComboBox()
  {
    ui.pos_axis_cb->addItem("x");
    ui.pos_axis_cb->addItem("y");
    ui.pos_axis_cb->addItem("z");
    ui.pos_axis_cb->setCurrentIndex(0);
    pos_axis_id_ = ui.pos_axis_cb->currentText().toStdString();
  }

  void MainWindow::initOrientationAxisComboBox()
  {
    ui.rot_axis_cb->addItem("x");
    ui.rot_axis_cb->addItem("y");
    ui.rot_axis_cb->addItem("z");
    ui.rot_axis_cb->setCurrentIndex(0);
    rot_axis_id_ = ui.rot_axis_cb->currentText().toStdString();
  }

  void MainWindow::respondPositionSliderChanged(int value)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pos_jog_value_ = 0.05 * value / 1000.0;
  }

  void MainWindow::respondPositionSliderReleased()
  {
    ui.pos_jog_hs->setValue(0);
  }

  void MainWindow::respondOrientationSliderChanged(int value)
  {
    boost::mutex::scoped_lock lock(mutex_);
    rot_jog_value_ = 0.05 * value / 1000.0;
  }

  void MainWindow::respondOrientationSliderReleased()
  {
    ui.rot_jog_hs->setValue(0);
  }

  void MainWindow::respondBaseSliderReleased()
  {
    ui.base_jog_vs->setValue(0);
  }

  void MainWindow::respondShoulderSliderReleased()
  {
    ui.shoulder_jog_vs->setValue(0);
  }

  void MainWindow::respondElbowSliderReleased()
  {
    ui.elbow_jog_vs->setValue(0);
  }

  void MainWindow::respondWrist1SliderReleased()
  {
    ui.wrist1_jog_vs->setValue(0);
  }

  void MainWindow::respondWrist2SliderReleased()
  {
    ui.wrist2_jog_vs->setValue(0);
  }

  void MainWindow::respondWrist3SliderReleased()
  {
    ui.wrist3_jog_vs->setValue(0);
  }

  void MainWindow::enableJogFrameMode(bool checked)
  {
    ROS_INFO("Jog frame button clicked!!");
    if(checked)
    {
      ui.jog_frame_enable_btn->setText("ON");
      writeLog("JOG FRAME MODE ENABLED");
    }
    else
    {
      ui.jog_frame_enable_btn->setText("OFF");
      writeLog("JOG FRAME MODE DISABLED");
    }
  }

  void MainWindow::enableJogJointMode(bool checked)
  {
    ROS_INFO("Jog Joint button clicked!!");
    if(checked)
    {
      ui.jog_joint_enable_btn->setText("ON");
      writeLog("JOG JOINT MODE ENABLED");
    }
    else
    {
      ui.jog_joint_enable_btn->setText("OFF");
      writeLog("JOG JOINT MODE DISABLED");
    }
  }

  /**
    action to perform on enable simulation mode
    - update the requested_mode to true 
    - disable the real robot mode/radio button
  **/
  void MainWindow::on_simModeRB(bool checked)
  {
    if(checked)
    {
      requested_mode = true;
      writeLog("SIMULATION MODE SET");

      ui.realModeRB->setAutoExclusive(false);
      ui.realModeRB->setChecked(false);
      ui.realModeRB->setAutoExclusive(true);

      add_reference_frames(true);
    }
  }

  /**
    action to perform on enable real robot mode
    - update the requested_mode to false 
    - disable the simulation mode/radio button
  **/
  void MainWindow::on_realModeRB(bool checked)
  {
    if(checked)
    {
      requested_mode = false;
      writeLog("REAL ROBOT MODE SET");

      ui.simModeRB->setAutoExclusive(false);
      ui.simModeRB->setChecked(false);
      ui.simModeRB->setAutoExclusive(true);

      add_reference_frames(false);
    }
  }

  void MainWindow::on_jointControlRB(bool checked)
  {
    if(checked && connection_flag)
    {
      qnode.init_joy_joint_control_mode();
    }
    else
    {
      writeLog("JOYSTICK JOINT MODE DISABLED");
      ROS_INFO("JOYSTICK JOINT MODE DISABLED");
    }
  }


  void MainWindow::on_toolControlRB(bool checked)
  {
    if(checked && connection_flag)
    {
      qnode.init_joy_frame_control_mode();
    }
    else
    {
      writeLog("JOYSTICK FRAME MODE DISABLED");
      ROS_INFO("JOYSTICK FRAME MODE DISABLED");
    }
  }

  void MainWindow::enableJogJoystickMode(bool checked)
  {
    ROS_INFO("Jog Joystick button clicked!!");
    if(checked)
    {
      ui.jog_joystick_enable_btn->setText("ON");

      // QProcess *exec = new QProcess(this);
      // exec->start("cd ~/home/anubhav1772/");
      // exec->execute("cd ~/pyenv/ros/bin/ && source activate && cd && cd catkin_ws; source ~/catkin_ws/devel/setup.bash; roslaunch trajectory_planning_gui joystick_control.launch");
      
      jogJoystickJointpublish();
      writeLog("JOG JOYSTICK MODE ENABLED");
    }
    else
    {
      ui.jog_joystick_enable_btn->setText("OFF");
      qnode.disable_joy_control_mode();
      writeLog("JOG JOYSTICK MODE DISABLED");
    }
  }

  void MainWindow::tabSelected()
  {
    if(ui.tabWidget->currentIndex()==0)
    {
      if(connection_flag)
      {
        writeLog("reading joint angles...");
        read_joint_angles();
      }
      else
      {
        writeLog("Unable to read joint angles. Please check connection with the arm!");
      }
    }
    else 
    {
      if(!connection_flag)
      {
        int ret = showErrorMessage("Connection Error", "Arm not connected!");
        switch (ret) 
        {
          case QMessageBox::Ok:
            {
              ui.tabWidget->setCurrentIndex(0);
            }
          default:
              // should never be reached
              break;
        }
      }
      else
      {
        if(ui.tabWidget->currentIndex()==1)
        {
          on_btn_read_kinematic_pose_clicked();
        }
        else if(ui.tabWidget->currentIndex()==2)
        {
          writeLog("Joint based trajectory planning.");
        }
        else if(ui.tabWidget->currentIndex()==3)
        {
          writeLog("Cartesian path planning.");
        }
      }
    }
  }

  void MainWindow::tabMainSelected()
  {
    if(ui.tab_widget_main->currentIndex()==0)
    {
      writeLog("HOME");
    }
    /**
    else if(ui.tab_widget_main->currentIndex()==8)
    {
      ROS_INFO("LOGGING TAB SELECTED");
      writeLog("Logging data!!");
    }
    **/
    else 
    {
      if(!connection_flag)
      {
        int ret = showErrorMessage("Connection Error", "Arm not connected!");
        switch (ret) 
        {
          case QMessageBox::Ok:
            {
              ui.tab_widget_main->setCurrentIndex(0);
            }
          default:
              // should never be reached
              break;
        }
      }
      else
      {
        if(ui.tab_widget_main->currentIndex()==1)
        {
          ROS_INFO("JOG TAB SELECTED");
          writeLog("JOG Mode");
        }
        else if(ui.tab_widget_main->currentIndex()==2)
        { 
          ROS_INFO("TEACH ARM TAB SELECTED");
          // ROS_INFO_STREAM("status: "<<qnode.get_teach_mode_status());
          ui.teachModeLog->setStyleSheet("QLabel { color : green; }");
          if(!qnode.get_teach_mode_status()) // false, teach mode is OFF, hence teachModeOn button is enabled
          {
            ui.teachModeLog->setText("Teach mode is OFF.");
          }
          else // true, teach mode is ON, hence teachModeOn button is disabled
          {
            int index = ui.geometryCB->currentIndex();
            ROS_INFO_STREAM("Teach Mode is ON with selected geometry "<<geometry[index]<<".");
            if(index==1||index==2||index==3)
            {
              ui.angleResDSB->setEnabled(true);          
            }
            else
            {
              ui.angleResDSB->setEnabled(false);
            }
          }
        }
        else if(ui.tab_widget_main->currentIndex()==3)
        {
          ROS_INFO("JOYSTICK BASED JOGGING");
        }
        else if(ui.tab_widget_main->currentIndex()==4)
        {
          ROS_INFO("ROBOT 3D VISUALIZATION TAB SELECTED");
          outfile << "ROBOT 3D VISUALIZATION TAB SELECTED.\n";
          ui.robot3d_verticalLayout->addWidget(qnode.attach_rviz_render_panel()); 
        }
        else if(ui.tab_widget_main->currentIndex()==5)
        {
          ROS_INFO("SENSOR DATA VISUALIZATION TAB SELECTED");
          qnode.init_cam_subscriber();
          writeLog("SENSOR DATA VISUALIZATION");
        }
        else if(ui.tab_widget_main->currentIndex()==5)
        {
          ROS_INFO("PROFILE TAB SELECTED");
          writeLog("Loaded saved profiles.");
        }
        else if(ui.tab_widget_main->currentIndex()==6)
        {
          ROS_INFO("SETTINGS TAB SELECTED");
          writeLog("Set parameters of the robotic arm.");
          readSettings();
          // add_reference_frames(qnode.get_robot_mode());
        }
      }
    }
  }

  void MainWindow::on_home(void)
  {
    ui.tab_widget_main->setCurrentIndex(0);
  }

  /**
    Function called on clicking refresh button.
  **/
  void MainWindow::on_refresh(void)
  {
    ROS_INFO_STREAM("refresh button clicked!!");
    // ROS_INFO_STREAM("SIM MODE: " <<qnode.get_robot_mode() <<"REQUESTED MODE: "<<requested_mode);

    if(qnode.get_robot_mode() == requested_mode)
    {
      QMessageBox msgBox;
      msgBox.setWindowTitle(QString::fromStdString("Switch Warning..."));
      msgBox.setText("Already in the same mode. No need to switch.");
      msgBox.exec();
      return;
    }
    else
    {
      QMessageBox msgBox;
      msgBox.setText("Pendant mode has been switched. Please save and restart the app.");
      msgBox.setInformativeText("Do you want to continue ?");
      msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
      msgBox.setDefaultButton(QMessageBox::Ok);
      int ret = msgBox.exec();
      switch (ret) 
      {
        case QMessageBox::Ok:
          {
            // Save was clicked
            std::string path = boost::filesystem::current_path().generic_string() + "/rosdep/meta.cache/__mode.txt";
            std::ofstream file(path);

            file << BoolToString(requested_mode);
            file.close();

            system("pkill roslaunch");
          }
        case QMessageBox::Cancel:
            // Cancel was clicked
            break;
        default:
            // should never be reached
            break;
      }
    }
    // close();
    // system("roslaunch trajectory_planning_gui trajectory_planning_gui");
  }

  void MainWindow::on_teachRobot(void)
  {
    ui.tab_widget_main->setCurrentIndex(2);
  }

  void MainWindow::on_settings(void)
  {
    ui.tab_widget_main->setCurrentIndex(6);
  }

  void MainWindow::on_info(void)
  {
    // ui.tab_widget_main->setCurrentIndex(8);
    // QtWebView view = new QtWebView(this);
    // view->load(QUrl("http://google.com/"));
    // view->show();
  }

  void MainWindow::on_btnVelScaling(void)
  {
    double value = ui.vel_scale_dsb->value();
    qnode.update_max_vel_scale_factor(value);
    writeLog(QString::fromStdString("Max velocity scaling factor changed to "+std::to_string(value)));
  }

  void MainWindow::on_btnAddWaypoint(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }
    else
    {
      if(waypoint_index==0)
      {
        qnode.call_query_state_client();
      }
      ++waypoint_index;
      waypoints_listModel->insertRow(waypoints_listModel->rowCount());
      QModelIndex index = waypoints_listModel->index(waypoints_listModel->rowCount()-1);

      waypoint_in_angle = qnode.getPresentJointAngle();
      waypoints.push_back(waypoint_in_angle);
      writeLog(QString::fromStdString("Added waypoint #"+std::to_string(waypoint_index)));

      std::stringstream ss;
      ss << "Waypoint "+std::to_string(waypoint_index)+" :[";
      for(int i=0;i<6;i++)
      {
        if(i!=0)
          ss << ",";
        // angle rounded to 3 decimal places
        ss << std::ceil(waypoint_in_angle[i]*1000.0) / 1000.0;
      }
      ss << "]";
      waypoints_listModel->setData(index, QString::fromStdString(ss.str()));
      writeLog(QString::fromStdString(ss.str()));
    }
  }

  void MainWindow::on_btnCancelTrajectory(void)
  {
    ROS_INFO("Cancelling Trajectory.");
    writeLog("Cancelling Trajectory.");
    qnode.cancel_trajectory();
  }

  void MainWindow::on_btnRunTrajectory(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }
    else
    {
      // execute angle based path planning
      if(waypoints.size()==0)
      {
        writeLog("No trajectory available. Nothing to run.");
      }
      else
      {
        int number_of_iterations = ui.iterationSpinBox->value();
        writeLog(QString::fromStdString("Number of iterations set to "+std::to_string(number_of_iterations)));
        ros::Duration(1).sleep(); // sleep for 1 seconds
        qnode.run_trajectory(number_of_iterations, waypoints, joint_velocities, joint_accelerations);
      }
    }
  }

  void MainWindow::on_btnSaveTrajectory()
  {
    /*! 
      Function for saving all the Way-Points(joint angle based waypoints) into yaml file. 
      This function opens a Qt Dialog where the user can set the name of the Way-Points file and the location. 
    */
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save Way Points"), ".yaml",
                                                    tr("Way Points (*.yaml);;All Files (*)"));

    if (fileName.isEmpty())
        return;
    else 
    {
      QFile file(fileName);
      if (!file.open(QIODevice::WriteOnly)) 
      {
          QMessageBox::information(this, tr("Unable to open file"), file.errorString());
          file.close();
          return;
      }

      YAML::Emitter out;
      out << YAML::BeginSeq;

      for(int i=0;i<waypoints.size();i++)
      {
        out << YAML::BeginMap;
        std::vector <double> points_vec;
        points_vec.push_back(waypoints[i][0]);
        points_vec.push_back(waypoints[i][1]);
        points_vec.push_back(waypoints[i][2]);
        points_vec.push_back(waypoints[i][3]);
        points_vec.push_back(waypoints[i][4]);
        points_vec.push_back(waypoints[i][5]);

        out << YAML::Key << "name";
        out << YAML::Value << (i+1);
        out << YAML::Key << "point";
        out << YAML::Value << YAML::Flow << points_vec;
        out << YAML::EndMap;
      }

      out << YAML::EndSeq;

      std::ofstream myfile;
      myfile.open (fileName.toStdString().c_str());
      myfile << out.c_str();
      myfile.close();
    }
  }

  void MainWindow::on_btnLoadTrajectory()
  {
    /*! 
      Slot that takes care of opening a previously saved Way-Points yaml file.
      Opens Qt Dialog for selecting the file, opens the file and parses the data. 
      After reading and parsing the data from the file, the information regarding the pose of the Way-Points is send to the RQT.
    */

    QString fileName =  QFileDialog::getOpenFileName(this,
                        tr("Open Way Points File"), "",
                        tr("Way Points (*.yaml);;All Files (*)"));

    if (fileName.isEmpty())
    {
      ui.tabWidget->setEnabled(true);
      // ui.progressBar->hide();
      return;
    }
    else 
    {
      ui.tabWidget->setEnabled(false);
      // ui.progressBar->show();
      QFile file(fileName);

      if (!file.open(QIODevice::ReadOnly)) {
          QMessageBox::information(this, tr("Unable to open file"), file.errorString());
          file.close();
          ui.tabWidget->setEnabled(true);
          // ui.progressBar->hide();
          return;
      }

      // clear list before loading new
      waypoint_index = 0;
      waypoints.clear();
      for(int i=ui.waypointslistView->model()->rowCount()-1; i>=0; --i)
      {
        ui.waypointslistView->model()->removeRow(i);
      }

      ROS_INFO_STREAM("Opening the file: "<<fileName.toStdString());
      std::ifstream fin(fileName.toStdString().c_str());

      YAML::Node doc;
      #ifdef HAVE_NEW_YAMLCPP
      doc = YAML::Load(fin);
      #else
      YAML::Parser parser(fin);
      parser.GetNextDocument(doc);
      #endif

      //define double for percent of completion
      double percent_complete;
      int end_of_doc = doc.size();

      for (size_t i = 0; i < end_of_doc; i++) {
        ++waypoint_index;
        std::string name;
        std::vector<double> joint_values;

        name = doc[i]["name"].as<std::string>();
        joint_values =  doc[i]["point"].as<std::vector<double>>();

        percent_complete = (i+1)*100/end_of_doc;
        // ui.progressBar->setValue(percent_complete);
        
        waypoints_listModel->insertRow(waypoints_listModel->rowCount());
        QModelIndex index = waypoints_listModel->index(waypoints_listModel->rowCount()-1);

        waypoints.push_back(joint_values);
        writeLog(QString::fromStdString("Added waypoint #"+std::to_string(waypoint_index)));

        std::stringstream ss;
        ss << "Waypoint "+std::to_string(waypoint_index)+" :[";
        for(int i=0;i<6;i++){
          if(i!=0)
            ss << ",";
          // angle rounded to 3 decimal places
          ss << std::ceil(joint_values[i]*1000.0) / 1000.0;
        }
        ss << "]";
        waypoints_listModel->setData(index, QString::fromStdString(ss.str()));
        writeLog(QString::fromStdString(ss.str()));
      }
      ui.tabWidget->setEnabled(true);
      // ui.progressBar->hide();
    }
  }
  
  void MainWindow::on_btnClearTrajectory(void)
  {
    if(waypoints.size()==0){
      writeLog("No trajectory available. Nothing to delete.");
    }else{
      waypoints.clear();
      for(int i=ui.waypointslistView->model()->rowCount()-1; i>=0; --i){
        ui.waypointslistView->model()->removeRow(i);
      }
      writeLog("[SUCCESS!!] Previous trajectory deleted.");
    }
  }

  void MainWindow::addAsWaypoint(void)
  {
    double x, y, z, rx, ry, rz;

    x = ui.doubleSpinBox_x->value();
    y = ui.doubleSpinBox_y->value();
    z = ui.doubleSpinBox_z->value();

    // roll
    rx = ui.doubleSpinBox_roll->value();
    // pitch
    ry = ui.doubleSpinBox_pitch->value();
    // yaw
    rz = ui.doubleSpinBox_yaw->value();

    tf::Transform point_pos(tf::Transform(tf::createQuaternionFromRPY(rx,ry,rz),tf::Vector3(x,y,z)));
    add_point(point_pos);
  }

  void MainWindow::addCurrentStateAsWaypoint(void)
  {
    // ROS_INFO_STREAM("x: "<< x << " y: " << y << " z: " << z);
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }

    std::vector<double> position = qnode.getPresentKinematicsPosition();
    double x, y, z, rx, ry, rz;

    x = position.at(0);
    y = position.at(1);
    z = position.at(2);
    // ROS_INFO_STREAM("X: "<< x << " Y: " << y << " Z: " << z);

    // roll
    rx = position.at(3);
    // pitch
    ry = position.at(4);
    // yaw
    rz = position.at(5);
    // ROS_INFO_STREAM("RX: "<< rx << " RY: " << ry << " RZ: " << rz);
    
    tf::Transform point_pos(tf::Transform(tf::createQuaternionFromRPY(rx,ry,rz),tf::Vector3(x,y,z)));
    add_point(point_pos);
  }

  void MainWindow::get_current_state()
  {
    std::vector<double> joint_angle = qnode.getPresentJointAngle();
    ui.txt_j1->setText(QString::number(RAD2DEG(joint_angle.at(0)),'f', 2));
    ui.txt_j2->setText(QString::number(RAD2DEG(joint_angle.at(1)),'f', 2));
    ui.txt_j3->setText(QString::number(RAD2DEG(joint_angle.at(2)),'f', 2));
    ui.txt_j4->setText(QString::number(RAD2DEG(joint_angle.at(3)),'f', 2));
    ui.txt_j5->setText(QString::number(RAD2DEG(joint_angle.at(4)),'f', 2));
    ui.txt_j6->setText(QString::number(RAD2DEG(joint_angle.at(5)),'f', 2));

    // ui.txt_grip->setText(QString::number(joint_angle.at(4),'f', 3));

    std::vector<double> position = qnode.getPresentKinematicsPosition();

    if(position.size() != 6)
      return;

    ui.txt_x->setText(QString::number(1000*position.at(0),'f', 2));
    ui.txt_y->setText(QString::number(1000*position.at(1),'f', 2));
    ui.txt_z->setText(QString::number(1000*position.at(2),'f', 2));

    ui.txt_roll->setText(QString::number(RAD2DEG(position.at(3)),'f', 2));
    ui.txt_pitch->setText(QString::number(RAD2DEG(position.at(4)),'f', 2));
    ui.txt_yaw->setText(QString::number(RAD2DEG(position.at(5)),'f', 2));
  }

  void MainWindow::writeLog(QString str)
  {
    ui.plainTextEdit_log->moveCursor (QTextCursor::End);
    ui.plainTextEdit_log->appendPlainText(str);
    outfile << "[ "+currentDateTime()+" ] "+str.toStdString()+"\n";
  }

  tf::Transform MainWindow::get_point_pos(void)
  {
    double x, y, z, rx, ry, rz;

    x = ui.x_dsb->value();
    y = ui.y_dsb->value();
    z = ui.z_dsb->value();

    rx = qnode.from_degrees(ui.roll_dsb->value());
    ry = qnode.from_degrees(ui.pitch_dsb->value());
    rz = qnode.from_degrees(ui.yaw_dsb->value());

    // create transform
    tf::Transform point_pos(tf::Transform(tf::createQuaternionFromRPY(rx,ry,rz),tf::Vector3(x,y,z)));
    return point_pos;
  }

  void MainWindow::add_point_UI(void)
  {
    /*! Function for adding new Way-Point from the RQT Widget.
        The user can set the position and orientation of the Way-Point by entering their values in the LineEdit fields.
        This function is connected to the btn_add_point button click() signal and sends the addPoint(point_pos) to inform the RViz enviroment that a new Way-Point has been added.
    */
    add_point(get_point_pos());
  }

  void MainWindow::add_point(const tf::Transform& point_pos)
  {
    if (!qnode.checkWayPointValidity(qnode.parseWayPoint(point_pos)))
    {
      writeLog(QString::fromStdString("IK solution not Found!"));
      return;
    }

    std::vector<tf::Transform >::iterator it_pos = std::find((waypoints_pos.begin()),(waypoints_pos.end()-1),point_pos);
    
    if (waypoints_pos.empty())
    {
      ROS_INFO("Adding first waypoint!");
      writeLog(QString::fromStdString("Adding first waypoint!"));

      waypoints_count++;

      waypoints_pos.push_back(point_pos);
      insert_row(point_pos, waypoints_count);
    }
    /*! 
      Check if we have points in the same position in the scene. If we do, do not add one and notify the RQT Widget so it can also add it to the TreeView.
    */
    else if((it_pos == (waypoints_pos.end())) || (point_pos.getOrigin() != waypoints_pos.at(waypoints_count-1).getOrigin())) // && (point_pos.getOrigin() != waypoints_pos.at(count_arrow-1).getOrigin()) //(it_pos == waypoints_pos.end()) &&
    {
        waypoints_count++;
        waypoints_pos.push_back(point_pos);

        ROS_INFO("Adding new waypoint!");
        writeLog(QString::fromStdString("Adding new waypoint!"));
        insert_row(point_pos, waypoints_count);
    }
    else
    {
        //if we have arrow, ignore adding new one and inform the user that there is arrow (waypoint at that location)
        ROS_INFO("There is already a arrow at that location, can't add new one!!");
        writeLog(QString::fromStdString("There is already a arrow at that location, can't add new one!!"));
    }
  }

  void MainWindow::insert_row(const tf::Transform& point_pos,const int count)
  {
    /*! Whenever we have a new Way-Point insereted either from the RViz or the RQT Widget 
        the the TreeView needs to update the information and insert new row that corresponds 
        to the new insered point.
        This function takes care of parsing the data recieved from the RViz or the RQT widget 
        and creating new row with the appropriate data format and Children. One for the position 
        giving us the current position of the Way-Point in all the axis. One child for the 
        orientation giving us the Euler Angles of each axis.
    */

    ROS_INFO("Inserting new row in the TreeView");
    writeLog(QString::fromStdString("Inserting new row in the TreeView"));
    
    QAbstractItemModel *model = ui.waypoints_treeView->model();

    // convert the quartenion to roll pitch yaw angle
    tf::Vector3 p = point_pos.getOrigin();
    tfScalar rx,ry,rz;
    point_pos.getBasis().getRPY(rx,ry,rz,1);

    writeLog(QString::fromStdString("x "+std::to_string(p.x())+"y "+std::to_string(p.y())+"z "+std::to_string(p.z())));
    writeLog(QString::fromStdString("roll "+std::to_string(rx)+"pitch "+std::to_string(ry)+"yaw "+std::to_string(rz)));

    if(count == 0)
    {
      model->insertRow(count, model->index(count, 0));
      model->setData(model->index(0,0,QModelIndex()),QVariant("add_waypoints"),Qt::EditRole);
    }
    else
    {
      if(!model->insertRow(count,model->index(count, 0)))  //&& count==0
      {
        return;
      }
      //set the strings of each axis of the position
      QString pos_x = QString::number(p.x());
      QString pos_y = QString::number(p.y());
      QString pos_z = QString::number(p.z());

      //repeat that with the orientation
      QString orient_x = QString::number(qnode.to_degrees(rx));
      QString orient_y = QString::number(qnode.to_degrees(ry));
      QString orient_z = QString::number(qnode.to_degrees(rz));

      model->setData(model->index(count,0),QVariant(count),Qt::EditRole);

      //add a child to the last inserted item. First add children in the treeview that
      //are just telling the user that if he expands them he can see details about the position and orientation of each point
      QModelIndex ind = model->index(count, 0);
      model->insertRows(0, 2, ind);

      QModelIndex chldind_pos = model->index(0, 0, ind);
      QModelIndex chldind_orient = model->index(1, 0, ind);
      model->setData(chldind_pos, QVariant("Position"), Qt::EditRole);
      model->setData(chldind_orient, QVariant("Orientation"), Qt::EditRole);

      //*****************************Set the children for the position**********************************************************
      //now add information about each child separately. For the position we have coordinates for X,Y,Z axis.
      //therefore we add 3 rows of information
      model->insertRows(0, 3, chldind_pos);

      //next we set up the data for each of these columns. First the names
      model->setData(model->index(0, 0, chldind_pos), QVariant("X:"), Qt::EditRole);
      model->setData(model->index(1, 0, chldind_pos), QVariant("Y:"), Qt::EditRole);
      model->setData(model->index(2, 0, chldind_pos), QVariant("Z:"), Qt::EditRole);

      //second we add the current position information, for each position axis separately
      model->setData(model->index(0, 1, chldind_pos), QVariant(pos_x), Qt::EditRole);
      model->setData(model->index(1, 1, chldind_pos), QVariant(pos_y), Qt::EditRole);
      model->setData(model->index(2, 1, chldind_pos), QVariant(pos_z), Qt::EditRole);
      //***************************************************************************************************************************

      //*****************************Set the children for the orientation**********************************************************
      //now we repeat everything again,similar as the position for adding the children for the orientation
      model->insertRows(0, 3, chldind_orient);

      //next we set up the data for each of these columns. First the names
      model->setData(model->index(0, 0, chldind_orient), QVariant("RX:"), Qt::EditRole);
      model->setData(model->index(1, 0, chldind_orient), QVariant("RY:"), Qt::EditRole);
      model->setData(model->index(2, 0, chldind_orient), QVariant("RZ:"), Qt::EditRole);

      //second we add the current position information, for each position axis separately
      model->setData(model->index(0, 2, chldind_orient), QVariant(orient_x), Qt::EditRole);
      model->setData(model->index(1, 2, chldind_orient), QVariant(orient_y), Qt::EditRole);
      model->setData(model->index(2, 2, chldind_orient), QVariant(orient_z), Qt::EditRole);
    }
  } 

  void MainWindow::remove_row()
  {
    /*! When the user deletes certain Way-Point either from the RViz or the RQT Widget the 
        TreeView needs to delete that particular row and update the state of the TreeWidget.
    */
    int row = ui.remove_pt_spin_box->value();

    // erase/remove the element at (row-1)th index from waypoints_pos vector
    waypoints_pos.erase(waypoints_pos.begin()+row-1);
    waypoints_count--;

    QAbstractItemModel *model = ui.waypoints_treeView->model();
    model->removeRow(row,QModelIndex());
    ROS_INFO_STREAM("deleting waypoint "<< row);
    writeLog(QString::fromStdString("deleting waypoint "+std::to_string(row)));

    for(int i=row;i<=model->rowCount();++i)
    {
      model->setData(model->index((i-1),0,QModelIndex()),QVariant((i-1)),Qt::EditRole);
    }
    
    // check how to properly set the selection
    ui.waypoints_treeView->selectionModel()->setCurrentIndex(model->index((model->rowCount()-1),0,QModelIndex()),QItemSelectionModel::ClearAndSelect);

    if(waypoints_count==0){
      QStringList headers;
      headers<<tr("Point")<<tr("Position (m)")<<tr("Orientation (deg)");
      ui.waypoints_treeView->setModel(new PointTreeModel(headers,QString::fromStdString("add_waypoint")));
    }
  }

  void MainWindow::add_point_before()
  {
    /*! When the user deletes certain Way-Point either from the RViz or the RQT Widget the 
        TreeView needs to delete that particular row and update the state of the TreeWidget.
    */
    int row = ui.add_pt_before_spin_box->value();

    if(waypoints_count==0){
      ROS_INFO("Waypoint list is empty. It's not possible to add row.");
      writeLog(QString::fromStdString("Waypoint list is empty. It's not possible to add row before"+std::to_string(row)));
      return;
    }
    else if(row>waypoints_count)
    {
      ROS_INFO("Waypoint list size is smaller than input row index.");
      writeLog(QString::fromStdString("Waypoint list size" +std::to_string(waypoints_count) + " is smaller than input row index "+std::to_string(row)));
      return;
    }
    else
    {
      tf::Transform point_pos = get_point_pos();
      waypoints_pos.insert(waypoints_pos.begin()+row-1, point_pos);
      waypoints_count++;
      
      insert_row(point_pos, row-1);
    }

    // erase/remove the element at (row-1)th index from waypoints_pos vector
    // waypoints_pos.insert(waypoints_pos.begin()+ros, );
    // waypoints_count--; 
  }

  /**
    If teach mode is enabled, then perform marker visualization as well as trajectory planning
    Else do only trajectory planning
  **/
  void MainWindow::execute_cartesian_trajectory(void)
  {
    // execute cartesian path planning
    if(waypoints_pos.size()==0)
    {
      writeLog("No cartesian trajectory available. Nothing to run.");
    }
    else 
    {
      if(ui.teachModeCB->isChecked())
      {
        /*
          TEACH MODE ENABLED
        */
        int index = ui.geometryCB->currentIndex();
        if(index==0)
        {
          /*
            STRAIGHT LINE TRAJECTORY
          */
          qnode.cartesian_curve(waypoints_pos, 0);
        }
        if(index==1)
        {
          /*
            ARC SHAPED TRAJECTORY
          */ 
          float angle_resolution = ui.angleResDSB->value(); 
          qnode.cartesian_curve(waypoints_pos, 1, angle_resolution);
        }
      }

      qnode.cartesianPathHandler(waypoints_pos);

      /**
      else
      {
        // int number_of_iterations = ui.iterationSpinBox->value();
        // writeLog(QString::fromStdString("Number of iterations set to "+std::to_string(number_of_iterations)));
        // ros::Duration(1).sleep(); // sleep for 1 seconds
        qnode.cartesianPathHandler(waypoints_pos);
        // if(success){
        //   writeLog(QString::fromStdString("Cartesian Path executed successfully"));
        // }else{
        //   writeLog(QString::fromStdString("Error while executing cartesian path"));
        // }
      }**/
    }
  }

  // Send a signal that a save the waypoints to a file.
  void MainWindow::savePointsToFile()
  {
    /*! 
      Function for saving all the Way-Points into yaml file. 
      This function opens a Qt Dialog where the user can set the name of the Way-Points file and the location. 
      Furthermore, it parses the way-points into a format that could be also loaded into the Plugin.
    */

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save Way Points"), ".yaml",
                                                    tr("Way Points (*.yaml);;All Files (*)"));

    if (fileName.isEmpty())
        return;
    else 
    {
      QFile file(fileName);
      if (!file.open(QIODevice::WriteOnly)) 
      {
          QMessageBox::information(this, tr("Unable to open file"), file.errorString());
          file.close();
          return;
      }

      YAML::Emitter out;
      out << YAML::BeginSeq;

      for(int i=0;i<waypoints_pos.size();i++)
      {
        out << YAML::BeginMap;
        std::vector <double> points_vec;
        points_vec.push_back(waypoints_pos[i].getOrigin().x());
        points_vec.push_back(waypoints_pos[i].getOrigin().y());
        points_vec.push_back(waypoints_pos[i].getOrigin().z());

        double rx, ry, rz;

        tf::Matrix3x3 m(waypoints_pos[i].getRotation());
        m.getRPY(rx, ry, rz,1);
        points_vec.push_back(RAD2DEG(rx));
        points_vec.push_back(RAD2DEG(ry));
        points_vec.push_back(RAD2DEG(rz));

        out << YAML::Key << "name";
        out << YAML::Value << (i+1);
        out << YAML::Key << "point";
        out << YAML::Value << YAML::Flow << points_vec;
        out << YAML::EndMap;
      }

      out << YAML::EndSeq;

      std::ofstream myfile;
      myfile.open (fileName.toStdString().c_str());
      myfile << out.c_str();
      myfile.close();
    }
  }

  // Send a signal that a load the waypoints from a file.
  void MainWindow::loadPointsFromFile()
  {
    /*! 
      Slot that takes care of opening a previously saved Way-Points yaml file.
      Opens Qt Dialog for selecting the file, opens the file and parses the data. 
      After reading and parsing the data from the file, the information regarding the pose of the Way-Points is send to the RQT.
    */

    QString fileName =  QFileDialog::getOpenFileName(this,
                        tr("Open Way Points File"), "",
                        tr("Way Points (*.yaml);;All Files (*)"));

    if (fileName.isEmpty())
    {
      ui.tabWidget->setEnabled(true);
      // ui.progressBar->hide();
      return;
    }
    else 
    {
      ui.tabWidget->setEnabled(false);
      // ui.progressBar->show();
      QFile file(fileName);

      if (!file.open(QIODevice::ReadOnly)) {
          QMessageBox::information(this, tr("Unable to open file"), file.errorString());
          file.close();
          ui.tabWidget->setEnabled(true);
          // ui.progressBar->hide();
          return;
      }

      //clear all the scene before loading all the new points from the file!!
      clearWayPoints();

      ROS_INFO_STREAM("Opening the file: "<<fileName.toStdString());
      std::ifstream fin(fileName.toStdString().c_str());

      YAML::Node doc;
      #ifdef HAVE_NEW_YAMLCPP
      doc = YAML::Load(fin);
      #else
      YAML::Parser parser(fin);
      parser.GetNextDocument(doc);
      #endif

      //define double for percent of completion
      double percent_complete;
      int end_of_doc = doc.size();

      for (size_t i = 0; i < end_of_doc; i++) {
        std::string name;
        geometry_msgs::Pose pose;
        tf::Transform pose_tf;
        
        double x,y,z,rx,ry,rz;
        name = doc[i]["name"].as<std::string>();
        x = doc[i]["point"][0].as<double>();
        y = doc[i]["point"][1].as<double>();
        z = doc[i]["point"][2].as<double>();
        rx = doc[i]["point"][3].as<double>();
        ry = doc[i]["point"][4].as<double>();
        rz = doc[i]["point"][5].as<double>();

        rx = DEG2RAD(rx);
        ry = DEG2RAD(ry);
        rz = DEG2RAD(rz);

        pose_tf = tf::Transform(tf::createQuaternionFromRPY(rx,ry,rz),tf::Vector3(x,y,z));

        percent_complete = (i+1)*100/end_of_doc;
        // ui.progressBar->setValue(percent_complete);
        add_point(pose_tf);
      }
      ui.tabWidget->setEnabled(true);
      // ui.progressBar->hide();

      if(waypoints_pos.size()==0)
      {
        writeLog("No cartesian trajectory available. Nothing to viualize.");
      }
      else
      {
        qnode.visualize_path_on_load(waypoints_pos);
      }
    }
  }

  // Send a signal that a clear all the waypoints.
  void MainWindow::clearWayPoints()
  {
    /* 
      Clear all the Way-Points from the TreeView.
      Also, deletes markers if exist.
    */
    qnode.deleteMarker();

    QAbstractItemModel *model = ui.waypoints_treeView->model();
    model->removeRows(0, model->rowCount());
    tf::Transform t;
    t.setIdentity();
    insert_row(t, 0); 

    waypoints_pos.clear();
    waypoints_count = 0;

    qnode.delete_visual_tools();
  }

  void MainWindow::on_comboBoxIndexChanged(void)
  {
    std::string imageTopicName = imageTopic[ui.imageTopicsComboBox->currentIndex()];
    writeLog(QString::fromStdString("Loading frames from "+imageTopicName+" topic."));
    qnode.set_camera_id(ui.imageTopicsComboBox->currentIndex());
    // writeLog(QString::fromStdString(std::to_string(ui.imageTopicsComboBox->currentIndex())));
    // ui.imageView->setPixmap(rgb_msg_to_pixmap(ui.imageTopicsComboBox->currentIndex()));
  }

  // void MainWindow::on_secondComboBoxIndexChanged(void)
  // {
  //   std::string imageTopicName = imageTopic[ui.imgComboBox->currentIndex()];
  //   writeLog(QString::fromStdString("Loading frames from "+imageTopicName+" topic."));
  // }

  void MainWindow::on_btn_set_clicked(void)
  {
    joint_velocities[ui.joint_names_comboBox->currentIndex()] = qnode.from_degrees(ui.speed_doubleSpinBox->value());
    joint_accelerations[ui.joint_names_comboBox->currentIndex()] = qnode.from_degrees(ui.acc_doubleSpinBox->value());
    writeLog(QString::fromStdString(joint_names_cb[ui.joint_names_comboBox->currentIndex()] + " control parameters (vel/accn) changed."));
    writeLog(QString::fromStdString("Updated Values: "));
    for(int i=0;i<6;i++)
    {
      writeLog(QString::fromStdString(joint_names_cb[i] + "<" + std::to_string(joint_velocities[i]) + " ," + std::to_string(joint_accelerations[i]) + ">"));
    }
  }

  void MainWindow::on_btn_reset_clicked(void)
  {
    for(int i=0;i<6;i++){
      joint_velocities[i] = 0.5;
      joint_accelerations[i] = 0.25;
    }
    writeLog(QString::fromStdString("control parameters (vel/accn) reset to original value."));
  }

  void MainWindow::on_jointNamesComboBoxIndexChanged(void)
  {
    joint_name_selected = joint_names_cb[ui.joint_names_comboBox->currentIndex()];
    writeLog(QString::fromStdString("Selected "+joint_name_selected+" ."));
  }

  void MainWindow::on_btn_init_pose_clicked(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }

    if(!qnode.setInitPose())
    {
      writeLog("[ERR!!] Failed to send service");
      return;
    }

    writeLog("Send joint angle to initial pose");
  }

  void MainWindow::on_btn_home_pose_clicked(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }

    if(!qnode.setHomePose())
    {
      writeLog("[ERR!!] Failed to send service");
      return;
    }
    writeLog("Send joint valid random pose");
  }

  void MainWindow::on_btn_random_pose_clicked(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }

    if(!qnode.setRandomPose())
    {
      writeLog("[ERR!!] Failed to send service");
      return;
    }
    writeLog("Send joint valid random pose");
  }

  void MainWindow::on_btn_up_pose_clicked(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }

    if(!qnode.setUpPose())
    {
      writeLog("[ERR!!] Failed to send service");
      return;
    }
    writeLog("Send joint up pose.");
  }

  void MainWindow::on_btn_gripper_open_clicked(void)
  {
    // op == 1 means open gripper action
    // value can decrease from 0.775 -> 0.04
    if(!qnode.operateGripper(1, 0.4))
    {
      writeLog("[ERROR] Failed to open gripper.");
      return;
    }
    writeLog("[SUCCESS] Open gripper action.");
  }

  void MainWindow::on_btn_gripper_close_clicked(void)
  {
    // op == 0 means close gripper action
    // value can decrease from 0.04 -> 0.775 
    if(!qnode.operateGripper(0, 0.775))
    {
      writeLog("[ERROR] Failed to close gripper.");
      return;
    }
    writeLog("[SUCCESS] Close gripper action.");
  }

  void MainWindow::on_btn_set_gripper_clicked(void)
  {
    double val = ui.doubleSpinBox_gripper->value();
    if(!qnode.operateGripper(2, val))
    {
      writeLog("[ERROR] Failed to set gripper to a desired value.");
      return;
    }
    writeLog("[SUCCESS] Gripper set to the desired value.");
  }

  void MainWindow::read_joint_angles(void)
  {
    writeLog("reading joint angles...");

    std::vector<double> joint_angle = qnode.getPresentJointAngle();
    ui.doubleSpinBox_j1->setValue(qnode.to_degrees(joint_angle.at(0)));
    ui.doubleSpinBox_j2->setValue(qnode.to_degrees(joint_angle.at(1)));
    ui.doubleSpinBox_j3->setValue(qnode.to_degrees(joint_angle.at(2)));
    ui.doubleSpinBox_j4->setValue(qnode.to_degrees(joint_angle.at(3)));
    ui.doubleSpinBox_j5->setValue(qnode.to_degrees(joint_angle.at(4)));
    ui.doubleSpinBox_j6->setValue(qnode.to_degrees(joint_angle.at(5)));
    // ui.doubleSpinBox_gripper->setValue(to_degrees(joint_angle.at(4)));

    ui.horizontalSlider_1->setValue(qnode.to_degrees(joint_angle.at(0)));
    ui.horizontalSlider_2->setValue(qnode.to_degrees(joint_angle.at(1)));
    ui.horizontalSlider_3->setValue(qnode.to_degrees(joint_angle.at(2)));
    ui.horizontalSlider_4->setValue(qnode.to_degrees(joint_angle.at(3)));
    ui.horizontalSlider_5->setValue(qnode.to_degrees(joint_angle.at(4)));
    ui.horizontalSlider_6->setValue(qnode.to_degrees(joint_angle.at(5)));
  }
  void MainWindow::on_btn_read_joint_angle_clicked(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }
    
    read_joint_angles();
  }

  void MainWindow::on_btn_send_joint_angle_clicked(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }

    std::vector<double> joint_angles;

    joint_angles.push_back(qnode.from_degrees(ui.doubleSpinBox_j1->value()));
    joint_angles.push_back(qnode.from_degrees(ui.doubleSpinBox_j2->value()));
    joint_angles.push_back(qnode.from_degrees(ui.doubleSpinBox_j3->value()));
    joint_angles.push_back(qnode.from_degrees(ui.doubleSpinBox_j4->value()));
    joint_angles.push_back(qnode.from_degrees(ui.doubleSpinBox_j5->value()));
    joint_angles.push_back(qnode.from_degrees(ui.doubleSpinBox_j6->value()));

    // if (!qnode.setJointSpacePath(joint_angle))
    // {
    //   writeLog("[ERR!!] Failed to send service");
    //   return;
    // }
    qnode.send_joint_data(joint_angles, joint_velocities, joint_accelerations);
    writeLog("Send joint angle");
  }

  void MainWindow::on_btn_read_kinematic_pose_clicked(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }

    std::vector<double> position = qnode.getPresentKinematicsPosition();
    ui.doubleSpinBox_x->setValue(position.at(0));
    ui.doubleSpinBox_y->setValue(position.at(1));
    ui.doubleSpinBox_z->setValue(position.at(2));

    ui.doubleSpinBox_roll->setValue(position.at(3));
    ui.doubleSpinBox_pitch->setValue(position.at(4));
    ui.doubleSpinBox_yaw->setValue(position.at(5));

    writeLog("Read task pose");
  }

  void MainWindow::on_btn_send_kinematic_pose_clicked(void)
  {
    if(!connection_flag)
    {
      showErrorMessage("Connection Error", "Arm not connected!");
      return;
    }

    std::vector<double> kinematics_pose;
    Eigen::Quaterniond temp_orientation;

    kinematics_pose.push_back(ui.doubleSpinBox_x->value());
    kinematics_pose.push_back(ui.doubleSpinBox_y->value());
    kinematics_pose.push_back(ui.doubleSpinBox_z->value());

    kinematics_pose.push_back(ui.doubleSpinBox_roll->value());
    kinematics_pose.push_back(ui.doubleSpinBox_pitch->value());
    kinematics_pose.push_back(ui.doubleSpinBox_yaw->value());

    if(!qnode.setTaskSpacePath(kinematics_pose))
    {
      writeLog("[ERR!!] Failed to send service");
      return;
    }
    writeLog("Send task pose");
  }

  void MainWindow::onTrajectoryPlanCancelled()
  {
    writeLog(QString::fromStdString("Stopped joint trajectory planning."));
  }

  void MainWindow::onTrajectoryExecutionFinished()
  {
    writeLog(QString::fromStdString("Finished executing joint trajectory planning."));
  }

  void MainWindow::onCartesianPathExecutionStarted()
  {
    writeLog(QString::fromStdString("Started cartesian path planning..."));
  }

  void MainWindow::onCartesianPathCompleted(double fraction)
  {
    writeLog(QString::fromStdString("Visualizing plan (cartesian path) " + std::to_string(fraction * 100.0) + " achieved."));
  }

  void MainWindow::onCartesianPathExecutionFinished()
  {
    writeLog(QString::fromStdString("Finished executing cartesian path planning."));
  }

  void MainWindow::ChangeSpinBox1(int sliderValue)
  {
    int value = convertSpinBoxValueToSlider(ui.doubleSpinBox_j1->value());
    if(value != sliderValue) 
    {
        ui.doubleSpinBox_j1->setValue((double)sliderValue);
    }
  }

  void MainWindow::ChangeSpinBox2(int sliderValue)
  {
    int value = convertSpinBoxValueToSlider(ui.doubleSpinBox_j2->value());
    if(value != sliderValue) 
    {
      ui.doubleSpinBox_j2->setValue((double)sliderValue);
    }
  }

  void MainWindow::ChangeSpinBox3(int sliderValue) 
  {
    int value = convertSpinBoxValueToSlider(ui.doubleSpinBox_j3->value());
    if(value != sliderValue) 
    {
      ui.doubleSpinBox_j3->setValue((double)sliderValue);
    }
  }

  void MainWindow::ChangeSpinBox4(int sliderValue) 
  {
    int value = convertSpinBoxValueToSlider(ui.doubleSpinBox_j4->value());
    if(value != sliderValue) 
    {
      ui.doubleSpinBox_j4->setValue((double)sliderValue);
    }
  }

  void MainWindow::ChangeSpinBox5(int sliderValue) 
  {
    int value = convertSpinBoxValueToSlider(ui.doubleSpinBox_j5->value());
    if(value != sliderValue) 
    {
      ui.doubleSpinBox_j5->setValue((double)sliderValue);
    }
  }

  void MainWindow::ChangeSpinBox6(int sliderValue) 
  {
    int value = convertSpinBoxValueToSlider(ui.doubleSpinBox_j6->value());
    if(value != sliderValue) 
    {
      ui.doubleSpinBox_j6->setValue((double)sliderValue);
    }
  }

  void MainWindow::ChangeVelScaleSpinBox(int sliderValue) 
  {
    int value = convertSpinBoxValueToSlider(ui.vel_scale_dsb->value()*100);
    if(value != sliderValue) 
    {
      ui.vel_scale_dsb->setValue((double)sliderValue/100.0);
    }
  }

  void MainWindow::ChangeSlider1(double spinBoxValue) 
  {
    if(ui.horizontalSlider_1->value() != (int)spinBoxValue)
    {
      ui.horizontalSlider_1->setValue((int)spinBoxValue);
    }
  }

  void MainWindow::ChangeSlider2(double spinBoxValue) 
  {
    if(ui.horizontalSlider_2->value() != (int)spinBoxValue)
    {
      ui.horizontalSlider_2->setValue((int)spinBoxValue);
    }
  }

  void MainWindow::ChangeSlider3(double spinBoxValue) 
  {
    if(ui.horizontalSlider_3->value() != (int)spinBoxValue)
    {
      ui.horizontalSlider_3->setValue((int)spinBoxValue);
    }
  }

  void MainWindow::ChangeSlider4(double spinBoxValue) 
  {
    if(ui.horizontalSlider_4->value() != (int)spinBoxValue)
    {
      ui.horizontalSlider_4->setValue((int)spinBoxValue);
    }
  }

  void MainWindow::ChangeSlider5(double spinBoxValue) 
  {
    if(ui.horizontalSlider_5->value() != (int)spinBoxValue)
    {
      ui.horizontalSlider_5->setValue((int)spinBoxValue);
    }
  }

  void MainWindow::ChangeSlider6(double spinBoxValue) 
  {
    if(ui.horizontalSlider_6->value() != (int)spinBoxValue)
    {
      ui.horizontalSlider_6->setValue((int)spinBoxValue);
    }
  }

  void MainWindow::ChangeVelScaleSlider(double spinBoxValue) 
  {
    if(ui.vel_scale_factor_slider->value() != (int)(spinBoxValue*100))
    {
      ui.vel_scale_factor_slider->setValue((int)(spinBoxValue*100));
    }
  }

  int MainWindow::convertSpinBoxValueToSlider(double value) 
  {
    return (int)round(value);
  }

  void MainWindow::setSliderStyle(QSlider *qslider)
  {
    qslider->setStyleSheet("QSlider::handle {"
                              "background: white;"
                              "border: 3px solid black;"
                              "width: 60px;"
                              "margin: -30px 0;"
                              "} "
                            "QSlider::sub-page {"
                              "background: rgb(164, 192, 2);"
                              "} "
                            "QSlider::add-page {"
                              "background: rgb(146,148,155);"
                              "} ");   
  }

  const char* MainWindow::BoolToString(bool b)
  {
    return b ? "true" : "false";
  }

  const std::string MainWindow::currentDateTime() 
  {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
  }

}  // namespace trajectory_planning_gui
