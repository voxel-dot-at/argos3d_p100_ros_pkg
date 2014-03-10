cmake_minimum_required(VERSION 2.8.3)
project(argos3d_p100)

## check required modules
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/external/cmake_modules")
find_package(PMD REQUIRED)

## preprocessor definitions
add_definitions(-DPMD_PLUGIN_DIR="${PMD_PLUGIN_DIR}/")

find_package(PCL REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf  
  pcl_ros
  pcl_conversions
  image_transport
  camera_info_manager
  camera_calibration_parsers
)

# PMD SDK Library
add_library (pmd_sdk src/sdk/pmdsdk2.cpp)


generate_dynamic_reconfigure_options(
  cfg/argos3d_p100.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sentis_m100
#  CATKIN_DEPENDS dynamic_reconfigure pcl_ros roscpp roslib rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PMD_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(argos3d_p100_node src/argos3d_p100_node.cpp)

add_dependencies(argos3d_p100_node ${PROJECT_NAME}_gencfg) 

target_link_libraries(argos3d_p100_node pmd_sdk pmdaccess2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

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
# install(TARGETS sentis_m100 sentis_m100_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sentis_m100.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
