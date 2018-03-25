cmake_minimum_required(VERSION 2.8.3)
project(modelica_ext_func)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  std_msgs
  message_generation
)

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
  Num.msg
  ModROSspring.msg
  ModROV.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  NumServer.srv
)

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
  LIBRARIES modelica_ext_func
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime joy
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#  src/${PROJECT_NAME}/modelica_ext_func.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/modelica_ext_func_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_modelica_ext_func.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## Tutorial runs, taken from ROS Tutorials
add_executable(talker src/tut/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker modelica_ext_func_generate_messages_cpp)

add_executable(listener src/tut/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener modelica_ext_func_generate_messages_cpp)

### Trials on connecting modelica to ROS

# First connection, directly handled controls, and communicated via a socket
add_executable(inet_socket src/spring/old/inet_socket.cpp)
target_link_libraries(inet_socket ${catkin_LIBRARIES})
add_dependencies(inet_socket modelica_ext_func_generate_messages_cpp)

# Second connection, acted as a relay between Modelica-C and the controller node, handled only one value
add_executable(inet_socket_relay src/spring/old/inet_socket_relay.cpp)
target_link_libraries(inet_socket_relay ${catkin_LIBRARIES})
add_dependencies(inet_socket_relay modelica_ext_func_generate_messages_cpp)

# Third connection first step, took in two values from socket via string concatenation, only published one value
add_executable(inet_socket_relay_array src/spring/old/inet_socket_relay_array.cpp)
target_link_libraries(inet_socket_relay_array ${catkin_LIBRARIES})
add_dependencies(inet_socket_relay_array modelica_ext_func_generate_messages_cpp)

# Third connection final, handled two values from socket, and passed via Num.msg custom message
add_executable(inet_socket_relay_array_personal src/spring/old/inet_socket_relay_array_personal.cpp)
target_link_libraries(inet_socket_relay_array_personal ${catkin_LIBRARIES})
add_dependencies(inet_socket_relay_array_personal modelica_ext_func_generate_messages_cpp)

# first iteration controller node, took in one value, and returned basic proportional control value
add_executable(modelica_controller src/spring/old/modelica_controller.cpp)
target_link_libraries(modelica_controller ${catkin_LIBRARIES})
add_dependencies(modelica_controller modelica_ext_func_generate_messages_cpp)

# second iteration controller node, takes in two values via Num.msg custom message, returns basic proportional control values
add_executable(modelica_controller_array src/spring/old/modelica_controller_array.cpp)
target_link_libraries(modelica_controller_array ${catkin_LIBRARIES})
add_dependencies(modelica_controller_array modelica_ext_func_generate_messages_cpp)

### Trials using joystick input to control values in Modelica model

# Main difference from previous iterations is custom message used is ModROSspring.msg, which uses an array of float64[2] for passing values
add_executable(inet_socket_relay_joy_input src/spring/inet_socket_relay_joy_input.cpp)
target_link_libraries(inet_socket_relay_joy_input ${catkin_LIBRARIES})
add_dependencies(inet_socket_relay_joy_input modelica_ext_func_generate_messages_cpp)

# Subscribes to additional node to recieve scaled joystick node values, to use as setpoint for proportional control.
# Uses ModROSspring.msg for passing values to relay node
add_executable(modelica_controller_joy_input src/spring/modelica_controller_joy_input.cpp)
target_link_libraries(modelica_controller_joy_input ${catkin_LIBRARIES})
add_dependencies(modelica_controller_joy_input modelica_ext_func_generate_messages_cpp)

# Recieves values from Joy node, scales values, and assigns desired axes values to the values within ModROSspring.msg
add_executable(mod_teleop src/spring/mod_teleop.cpp)
target_link_libraries(mod_teleop ${catkin_LIBRARIES})
add_dependencies(mod_teleop modelica_ext_func_generate_messages_cpp)

### Using Joystick to control ROV model propeller action

# Waits to recieve request message from socket to send in control values
# Communicates with controller via ModROV.msg, which contains a float64[6] array
add_executable(inet_socket_ROV_relay_joy src/rov/inet_socket_ROV_relay_joy.cpp)
target_link_libraries(inet_socket_ROV_relay_joy ${catkin_LIBRARIES})
add_dependencies(inet_socket_ROV_relay_joy modelica_ext_func_generate_messages_cpp)

# First iteration of Joystick value control for the ROV
# Directly subscribes to the Joy node, scales values, and assigns appropriate axes values to ModROV.msg values
add_executable(rov_joystick_basic_control src/rov/rov_joystick_basic_control.cpp)
target_link_libraries(rov_joystick_basic_control ${catkin_LIBRARIES})
add_dependencies(rov_joystick_basic_control modelica_ext_func_generate_messages_cpp)

# Second iteration of Joystick value control for the ROV
# As there is no actual feedback based control happening yet, there is no need for a separate mod_teleop-like node
# This focuses on passing tuned values through ModROV.msg based on the values coming from the joystick
add_executable(rov_joystick_tuned_control src/rov/rov_joystick_tuned_control.cpp)
target_link_libraries(rov_joystick_tuned_control ${catkin_LIBRARIES})
add_dependencies(rov_joystick_tuned_control modelica_ext_func_generate_messages_cpp)