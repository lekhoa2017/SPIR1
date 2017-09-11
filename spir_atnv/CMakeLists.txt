cmake_minimum_required(VERSION 2.8.3)
project(spir_atnv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs cmake_modules message_generation tf)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.2 REQUIRED)

include(FindPackageHandleStandardArgs)

find_path(qpOASES_INCLUDEDIR
          NAMES qpOASES.hpp
          HINTS "${qpOASES_SOURCE_DIR}"
                ENV qpOASES_SOURCE_DIR
          PATH_SUFFIXES include)
find_library(qpOASES_LIB
             NAMES qpOASES
             HINTS "${qpOASES_BINARY_DIR}"
                   ENV qpOASES_BINARY_DIR
             PATH_SUFFIXES lib
                           libs)

set(qpOASES_INCLUDE_DIRS ${qpOASES_INCLUDEDIR})
set(qpOASES_LIBRARIES ${qpOASES_LIB})

find_package_handle_standard_args(qpOASES DEFAULT_MSG qpOASES_LIBRARIES
                                                      qpOASES_INCLUDE_DIRS)
set(qpOASES_FOUND ${QPOASES_FOUND})



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
add_message_files(
  FILES
  ThrustStamped.msg
  PileStateStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES thruster_description
  CATKIN_DEPENDS message_runtime
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include ${catkin_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${qpOASES_INCLUDE_DIRS} 
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library (controlLib src/class_files/control.cpp src/class_files/rise_pid.cpp)
## Declare a C++ library
# add_library(thruster_description
#   src/${PROJECT_NAME}/thruster_description.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(thruster_description ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(atnv_control src/nodes/atnv_control.cpp src/class_files/robust_integral_control_full_dof.cpp src/class_files/calc_coeff_matrix.cpp src/class_files/kalman.cpp)
add_executable(imu_control src/nodes/imu_control.cpp src/class_files/robust_integral_control.cpp src/class_files/calc_coeff_matrix.cpp src/class_files/kalman.cpp)
add_executable(control_node src/nodes/control_node.cpp src/class_files/model_do.cpp src/class_files/rise_pid.cpp src/class_files/calc_coeff_matrix.cpp src/class_files/kinematics.cpp src/class_files/control.cpp)
add_executable(reference_generator src/nodes/reference_generator.cpp)
add_executable(PID_control1 src/nodes/PID_control1.cpp)

add_executable(Open_loop_test1 src/nodes/Open_loop_test1.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(thruster_description_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(atnv_control spir_atnv_generate_messages_cpp)
add_dependencies(imu_control spir_atnv_generate_messages_cpp)
add_dependencies(control_node spir_atnv_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(reference_generator spir_atnv_generate_messages_cpp)
add_dependencies(controlLib spir_atnv_generate_messages_cpp)

add_dependencies(PID_control1 spir_atnv_generate_messages_cpp)

add_dependencies(Open_loop_test1 spir_atnv_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(atnv_control ${catkin_LIBRARIES})
target_link_libraries(imu_control ${catkin_LIBRARIES})
target_link_libraries(control_node ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} )
target_link_libraries(reference_generator ${catkin_LIBRARIES})
target_link_libraries(PID_control1 ${catkin_LIBRARIES} )

target_link_libraries(Open_loop_test1 ${catkin_LIBRARIES} )

# target_link_libraries(thruster_description_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS thruster_description thruster_description_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_thruster_description.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

