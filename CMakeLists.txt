cmake_minimum_required(VERSION 2.8.3)
project(mkp_pcd)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  geometry_msgs
  pcl_ros
  tf
  roslib
  message_generation
  trajectory_msgs
  libpointmatcher_ros
  libpointmatcher
)

find_package(libpointmatcher CONFIG)
find_package( orocos_kdl REQUIRED )
find_package( Eigen3 REQUIRED )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   DIRECTORY
   srv
   FILES
   targeting.srv
   registration_clear.srv
   registration_merge_and_output.srv
   registration_process_registration.srv
   registration_take_view.srv
   trigger.srv
   movj.srv
   FindWidthOrHeight.srv
 )


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   trajectory_msgs
   geometry_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mkp_pcd
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    libpointmatcher
    libpointmatcher_ros
    trajectory_msgs
    geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
  include/
)

find_package(PCL 1.2 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#------------------targeting---------------------------
add_executable(${PROJECT_NAME}_targeting_node 
src/point_cloud_targeting_main.cpp
src/point_cloud_targeting.cpp
include/point_cloud_targeting.h
)

add_dependencies(${PROJECT_NAME}_targeting_node ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_targeting_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#------------------stitching---------------------------
add_executable(${PROJECT_NAME}_registration_node 
src/point_cloud_registration_main.cpp
src/point_cloud_registration.cpp
include/point_cloud_registration.h
)
add_dependencies(${PROJECT_NAME}_registration_node ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_registration_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${libpointmatcher_LIBRARIES})

#------------------color saving---------------------------

add_executable(${PROJECT_NAME}_saveColor_node 
src/color_point_cloud_save_main.cpp
)
add_dependencies(${PROJECT_NAME}_saveColor_node ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_saveColor_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#------------------ultrasound_main------------------------

add_executable(${PROJECT_NAME}_ultrasound_main  
src/ultrasound_main.cpp
)
add_dependencies(${PROJECT_NAME}_ultrasound_main ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_ultrasound_main ${catkin_LIBRARIES})

#------------------ros_iiwa_publisher--------------------

add_executable(${PROJECT_NAME}_node_fkpublisher src/fkpublisher.cpp)
target_link_libraries(${PROJECT_NAME}_node_fkpublisher ${catkin_LIBRARIES} ${PCL_LIBRARIES} orocos-kdl)


#********************* MOT_movJServerCallback   *******************

add_executable(${PROJECT_NAME}_node_movJServerCallback 
  src/MOT_movjcallback.cpp )

target_link_libraries(${PROJECT_NAME}_node_movJServerCallback  
  ${catkin_LIBRARIES} 
)

add_dependencies(${PROJECT_NAME}_node_movJServerCallback ${PROJECT_NAME}_gencpp)

#------------------------SegmentPhantomWidth-------------------------

add_executable(${PROJECT_NAME}_SegmentPhantomWidth_node 
src/SegmentPhantomWidth.cpp
)
add_dependencies(${PROJECT_NAME}_SegmentPhantomWidth_node ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_SegmentPhantomWidth_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mkp_pcd.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
