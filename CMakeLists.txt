cmake_minimum_required(VERSION 2.8.3)
project(voxel_trajectory)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  geometry_msgs
  nav_msgs
  quadrotor_msgs
 # mavlink_message
  laser_geometry
  message_generation
  voxel_map
  pcl_ros
)

find_package(PCL 1.7 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIRS})

# OMPL
find_package(OMPL)
include_directories(
    ${OMPL_INCLUDE_DIR}
)

# for PCL
#find_package(PCL REQUIRED common io)
#include_directories(
#    ${PCL_INCLUDE_DIRS}
#)

include_directories(VoxelTrajectory/include)

# for c++
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g ${CMAKE_CXX_FLAGS}")

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   CheckObstacleByPoints.srv
#   Service1.srv
#   Service2.srv
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
#   std_msgs
#   senesor_msgs
# )

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
  INCLUDE_DIRS include
  LIBRARIES octomap_ voxelgraph_  trajectorygenerator_
  CATKIN_DEPENDS roscpp 
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(voxel_trajecory
#   src/${PROJECT_NAME}/voxel_trajecory.cpp
# )

add_library(octomap_		src/voxel_trajectory/octomap.cpp)
add_library(voxelgraph_		src/voxel_trajectory/voxel_graph.cpp)
add_library(trajectorygenerator_	src/voxel_trajectory/trajectory_generator_fast.cpp)
add_library(trajectorygenerator_no_inflation	src/voxel_trajectory/trajectorygenerator_no_inflation.cpp)

## Declare a cpp executable
# add_executable(voxel_trajecory_node src/voxel_trajecory_node.cpp)

#add_executable(test_voxel src/test_voxel.cpp)

#add_executable(test_server src/test_server.cpp)

add_executable(mission_commander_node src/mission_commander_node.cpp)

#add_executable(voxel_trajectory_generator_node src/trajectory_generator_node.cpp)

#add_executable(voxel_bridge_node src/voxel_bridge_node.cpp)

#add_executable(voxel_bridge_enhanced_node src/voxel_bridge_enhanced_node.cpp)

add_executable(voxel_set_obstacle_node src/voxel_set_obstacle_node.cpp)

#add_executable(trajectory_server_node src/trajectory_server_node.cpp)

add_executable(test_limited_sensing_node src/test_limited_sensing_node.cpp)

add_executable(body_tf src/body_tf.cpp)

add_executable(grid_trajectory_server_node src/grid_trajectory_server_node.cpp)

add_executable(grid_trajectory_generator_node src/grid_trajectory_generator_node.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(voxel_trajecory_node voxel_trajecory_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(voxel_trajecory_node
#   ${catkin_LIBRARIES}
# )

#target_link_libraries( test_voxel
#	#${catkin_LIBRARIES}
#	octomap_
#	voxelgraph_
#	trajectorygenerator_
#
#	ooqpgensparse
#	ooqpsparse
#	ooqpgondzio
#	ooqpbase
#	blas
#	ma27
#	gfortran
#)

#target_link_libraries( test_server
#	#${catkin_LIBRARIES}
#	octomap_
#	voxelgraph_
#	trajectorygenerator_
#
#	ooqpgensparse
#	ooqpsparse
#	ooqpgondzio
#	ooqpbase
#	blas
#	ma27
#	gfortran
#)

target_link_libraries(body_tf
    ${catkin_LIBRARIES}
)

target_link_libraries(octomap_ voxelgraph_)

target_link_libraries(test_limited_sensing_node
    ${catkin_LIBRARIES}
    octomap_
)

#target_link_libraries( voxel_trajectory_generator_node
#	${catkin_LIBRARIES}
#	${PCL_COMMON_LIBRRIES}
#	${PCL_IO_LIBRARIES}
#       
#	octomap_
#	trajectorygenerator_
#
#	ooqpgensparse
#	ooqpsparse
#	ooqpgondzio
#	ooqpbase
#	blas
#	ma27
#	gfortran
#)


target_link_libraries( mission_commander_node
	${catkin_LIBRARIES}
#	${PCL_COMMON_LIBRRIES}
#	${PCL_IO_LIBRARIES}

	octomap_
	voxelgraph_
	trajectorygenerator_

	ooqpgensparse
	ooqpsparse
	ooqpgondzio
	ooqpbase
	blas
	ma27
	gfortran
)

#target_link_libraries( voxel_bridge_node
#	${catkin_LIBRARIES}
#)

#target_link_libraries( voxel_bridge_enhanced_node
#	${catkin_LIBRARIES}
#)

target_link_libraries(voxel_set_obstacle_node
    ${catkin_LIBRARIES}
)

target_link_libraries(grid_trajectory_generator_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}

    octomap_
    trajectorygenerator_

    ooqpgensparse
    ooqpsparse
    ooqpgondzio
    ooqpbase
    blas
    ma27
    gfortran
    voxel_map
)

target_link_libraries(grid_trajectory_server_node
    ${catkin_LIBRARIES}
)

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
# install(TARGETS voxel_trajecory voxel_trajecory_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_voxel_trajecory.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
