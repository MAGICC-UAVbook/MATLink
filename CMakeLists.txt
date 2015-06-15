cmake_minimum_required(VERSION 2.8.3)
project(matlink_mex_from_mavros)

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
#SET(CMAKE_STATIC_LINKER_FLAGS "-fPIC")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#set_target_properties(mavros PROPERTIES LINK_FLAGS -fPIC)
add_dependencies(mavros
  #mavros_generate_messages_cpp
  mavconn
)
target_link_libraries(mavros
  mavconn
  #${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


##############
# standalone #
##############

## Declare a cpp executable
add_executable(matlink_test
  src/matlink_test.cpp
)
add_dependencies(matlink_test
  mavros
  mavconn
)
target_link_libraries(matlink_test
  mavros
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
)

#######
# MEX #
#######

# compile mex
set(CPP_FILE MATLink)

# set up matlab libraries
set(CPP_FILE_NAME src/${CPP_FILE}.cpp)
INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR} ${SIMULINK_INCLUDE_DIR})
add_library(${CPP_FILE} SHARED ${CPP_FILE_NAME} ${CMAKE_SOURCE_DIR}/Matlabdef.def
  src/mavconn/mavconn_interface.cpp
  src/mavconn/mavconn_serial.cpp
  src/mavconn/mavconn_udp.cpp
  src/mavconn/mavconn_tcp.cpp
  src/lib/uas.cpp
  src/lib/mavros.cpp
  include/mavros/mavros.h
  )
#add_dependencies(${CPP_FILE} mavros mavconn)
target_link_libraries(${CPP_FILE} ${MATLAB_LIBRARIES} ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)#mavros)

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
