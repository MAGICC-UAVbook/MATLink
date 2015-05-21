cmake_minimum_required(VERSION 2.8.3)
project(matlink_mex_from_mavros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS
#  diagnostic_msgs
#  diagnostic_updater
#  message_generation
#  message_runtime
#  pluginlib
#  roscpp
#  sensor_msgs
#  std_msgs
#  std_srvs
#  tf
#  angles
#)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
include(EnableCXX11)
include(MavrosMavlink)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules) # add FindMatlab module

add_definitions(/DMATLAB_MEX_FILE) #define matlab macros
add_definitions(/DMX_COMPAT_32)

find_package(Matlab REQUIRED)

IF(MATLAB_FOUND)
    message(STATUS "MATLAB Found, MATLAB MEX will be compiled.")
ELSE(MATLAB_FOUND)
    MESSAGE("MATLAB not found...nothing will be built.")
ENDIF(MATLAB_FOUND)

#SET(GCC_LINK_FLAGS "-fPIC")
#SET( CMAKEMODULE_LINKER_FLAGS  "${CMAKE_MODULE_LINKER_FLAGS} ${GCC_LINK_FLAGS}" )

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
#add_message_files(
#  FILES
#  Mavlink.msg
#  Waypoint.msg
#  WaypointList.msg
#  BatteryStatus.msg
#  State.msg
#  RCIn.msg
#  RCOut.msg
#  RadioStatus.msg
#  OverrideRCIn.msg
#  VFR_HUD.msg
#  FileEntry.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  ParamSet.srv
#  ParamGet.srv
#  ParamPull.srv
#  ParamPush.srv
#  WaypointSetCurrent.srv
#  WaypointClear.srv
#  WaypointPull.srv
#  WaypointPush.srv
#  WaypointGOTO.srv
#  CommandLong.srv
#  CommandInt.srv
#  CommandBool.srv
#  CommandHome.srv
#  CommandTOL.srv
#  StreamRate.srv
#  SetMode.srv
#  FileOpen.srv
#  FileClose.srv
#  FileRead.srv
#  FileWrite.srv
#  FileList.srv
#  FileRemove.srv
#  FileMakeDir.srv
#  FileRemoveDir.srv
#  FileTruncate.srv
#  FileRename.srv
#  FileChecksum.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  diagnostic_msgs
#  sensor_msgs
#  std_msgs
#)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mavconn mavros
#  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater pluginlib roscpp sensor_msgs std_msgs tf geometry_msgs
#  DEPENDS Boost
#  CFG_EXTRAS mavros-extras.cmake
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# TODO: Check names of system library include directories (boost)
include_directories(
  include
#  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mavconn
  src/mavconn/mavconn_interface.cpp
  src/mavconn/mavconn_serial.cpp
  src/mavconn/mavconn_udp.cpp
  src/mavconn/mavconn_tcp.cpp
)
#add_dependencies(mavconn
#  mavros_generate_messages_cpp
#)
target_link_libraries(mavconn
  #${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

add_library(mavros
  src/lib/uas.cpp
  src/lib/mavros.cpp
#  src/lib/mavlink_diag.cpp
)
add_dependencies(mavros
  #mavros_generate_messages_cpp
  mavconn
)
set_target_properties(mavros PROPERTIES LINK_FLAGS -fPIC)
target_link_libraries(mavros
  mavconn
  #${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#add_library(mavros_plugins
#  src/plugins/dummy.cpp
#  src/plugins/sys_status.cpp
#  src/plugins/sys_time.cpp
#  src/plugins/imu_pub.cpp
#  src/plugins/gps.cpp
#  src/plugins/param.cpp
#  src/plugins/waypoint.cpp
#  src/plugins/rc_io.cpp
#  src/plugins/command.cpp
#  src/plugins/local_position.cpp
#  src/plugins/global_position.cpp
#  src/plugins/setpoint_position.cpp
#  src/plugins/safety_area.cpp
#  src/plugins/3dr_radio.cpp
#  src/plugins/setpoint_velocity.cpp
#  src/plugins/setpoint_accel.cpp
#  src/plugins/setpoint_attitude.cpp
#  src/plugins/vfr_hud.cpp
#  src/plugins/ftp.cpp
#  src/plugins/distance_pub.cpp
#)
#add_dependencies(mavros_plugins
#  mavros
#)
#target_link_libraries(mavros_plugins
#  mavconn
#  mavros
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

## Declare a cpp executable
#add_executable(matlink_test
#  src/matlink_test.cpp
#)
#add_dependencies(matlink_test
#  mavros
#  mavconn
#)
#target_link_libraries(matlink_test
#  mavros
##  ${catkin_LIBRARIES}
##  ${Boost_LIBRARIES}
#)


# compile mex
set(CPP_FILE MATLink)

# set up matlab libraries
set(CPP_FILE_NAME src/${CPP_FILE}.cpp)
INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR} ${SIMULINK_INCLUDE_DIR})
add_library(${CPP_FILE} SHARED ${CPP_FILE_NAME} ${CMAKE_SOURCE_DIR}/Matlabdef.def)
add_dependencies(${CPP_FILE} mavros mavconn)
target_link_libraries(${CPP_FILE} ${MATLAB_LIBRARIES} mavros)

# 32-bit or 64-bit mex
if(WIN32)
  if (CMAKE_CL_64)
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexw64)
  else(CMAKE_CL_64)
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexw32)
  endif(CMAKE_CL_64)
else(WIN32)
  if (CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexa64 PREFIX "")
  else(CMAKE_SIZEOF_VOID_P MATCHES "8")
      SET_TARGET_PROPERTIES(${CPP_FILE} PROPERTIES SUFFIX .mexglx PREFIX "")
  endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
endif(WIN32)

# install to /bin by default
#install(TARGETS ${CPP_FILE} DESTINATION ../bin)

#add_executable(gcs_bridge
#  src/gcs_bridge.cpp
#)
#add_dependencies(gcs_bridge
#  mavconn
#)
#target_link_libraries(gcs_bridge
#  mavconn
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)
#catkin_install_python(PROGRAMS
#  ${SCRIPTS}
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
#install(TARGETS mavconn mavros mavros_node gcs_bridge
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#  mavros_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

## Install cmake files (thanks to cmake_modules package)
#install(DIRECTORY cmake/Modules
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(mavconn-test test/test_mavconn.cpp)
if(TARGET mavconn-test)
  target_link_libraries(mavconn-test mavconn)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# vim: ts=2 sw=2 et:
