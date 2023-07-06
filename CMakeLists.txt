cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planning_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  interactive_markers
  geometric_shapes
  actionlib
  control_msgs
  trajectory_msgs
  cv_bridge
  rviz
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Multimedia REQUIRED)
find_package(Qt5MultimediaWidgets REQUIRED)
find_package(Boost REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	JogFrame.msg
  JogJoint.msg
  MoveJoint.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
 )

set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
  src/point_tree_model.cpp
  src/point_tree_item.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/TabProxy.h
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/qnode.hpp
  include/${PROJECT_NAME}/point_tree_model.hpp
  include/${PROJECT_NAME}/point_tree_item.hpp
  include/${PROJECT_NAME}/json.hpp
  
)

set(${PROJECT_NAME}_RESOURCES
  resources/images.qrc
)

set(${PROJECT_NAME}_UIS
  ui/main_window.ui
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

if(NOT EXISTS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
endif()

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp 
    std_msgs 
    sensor_msgs 
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    cv_bridge
  DEPENDS EIGEN3
)

qt5_wrap_cpp(${PROJECT_NAME}_MOCS     ${${PROJECT_NAME}_HDRS})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_RESOURCES})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${trajectory_planning_gui_INCLUDE_DIRECTORIES})

qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

## https://stackoverflow.com/questions/30535933/eigen-gcc-5-class-stdbinder2nd-is-deprecated
## supress deprecation warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

################################################################################
# Build
################################################################################
include_directories(
  include
  ${trajectory_planning_gui_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(trajectory_planning_gui ${trajectory_planning_gui_SRCS} ${trajectory_planning_gui_MOCS} ${trajectory_planning_gui_UIS_H} ${trajectory_planning_gui_RCC})
add_dependencies(trajectory_planning_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_planning_gui Qt5::Widgets Qt5::Core Qt5::Multimedia Qt5::MultimediaWidgets ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

# linking the yaml-cpp library; used while reading waypoints from yaml file
target_link_libraries(${PROJECT_NAME} yaml-cpp)

#################################################################################
## Install
#################################################################################
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

