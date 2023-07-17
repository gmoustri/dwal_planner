cmake_minimum_required(VERSION 3.0.2)
project(dwal_planner)

## Compile as C++11, supported in ROS Kinetic and newer
##add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
  message_generation
  tf 
  tf2
  tf2_ros
  costmap_2d
  base_local_planner
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Cluster_Group.msg
   Path.msg
   Path_Cluster.msg
   Pose2D_32.msg
   Sampled_Cluster.msg
   Sampled_Path.msg
   Thin_Cluster.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   toggleClusters.srv
   toggleSlice.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs 
   sensor_msgs
   std_msgs
   trajectory_msgs
   visualization_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################set(CMAKE_BUILD_TYPE Debug)
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cluster
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs tf tf2 tf2_ros std_msgs trajectory_msgs visualization_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(dwal_cluster
   src/libdwal_cluster.cpp
 )
 
 add_dependencies(dwal_cluster dwal_planner_generate_messages_cpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide


add_executable(dwal_clustering src/clustering.cpp)
add_executable(dwal_generator src/trajectory_generator.cpp)

target_link_libraries(dwal_clustering dwal_cluster ${catkin_LIBRARIES})
target_link_libraries(dwal_generator dwal_cluster ${catkin_LIBRARIES})


