# ============================================================
#
# This file is a part of CoSiMA (CogIMon) project
#
# Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
#
# This file may be licensed under the terms of the
# GNU Lesser General Public License Version 3 (the ``LGPL''),
# or (at your option) any later version.
#
# Software distributed under the License is distributed
# on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
# express or implied. See the LGPL for the specific language
# governing rights and limitations.
#
# You should have received a copy of the LGPL along with this
# program. If not, go to http://www.gnu.org/licenses/lgpl.html
# or write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# The development of this software was supported by:
#   CoR-Lab, Research Institute for Cognition and Robotics
#     Bielefeld University
#
# ============================================================

cmake_minimum_required(VERSION 2.8.3)
project(cosima-controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#SET(CMAKE_BUILD_TYPE RelWithDebInfo)
# SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  kdl_parser
  cosima_msgs
)

if (NOT OROCOS-RTT_FOUND)
  message (STATUS "OROCOS RTT not found through catkin. Check find_package alternative")
  find_package(OROCOS-RTT REQUIRED)
  if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
  endif()
  
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
endif()

find_package(orocos_kdl REQUIRED)
link_directories(${orocos_kdl_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
message(STATUS "yaml-cpp version: ${YAML_CPP_VERSION}")
message(STATUS "YAML_CPP_INCLUDE_DIR: ${YAML_CPP_INCLUDE_DIR}")
message(STATUS "YAML_CPP_LIBRARIES: ${YAML_CPP_LIBRARIES}")
include_directories(BEFORE SYSTEM ${YAML_CPP_INCLUDE_DIR})
add_definitions(${YAML_CPP_CFLAGS})

set(OpenSoT_DIR /opt/xbot/lib/cmake/OpenSoT)
set(XBotInterface_DIR /opt/xbot/lib/cmake/XBotInterface)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/xbot)

find_package(OpenSoT REQUIRED)
if (NOT OpenSoT_FOUND)
  message (FATAL_ERROR "\nCould not find OpenSoT. Won't build OpenSoT related components.")
endif()
message(STATUS "OpenSoT_INCLUDE_DIRS: ${OpenSoT_INCLUDE_DIRS}")
message(STATUS "XBotInterface_INCLUDE_DIRS: ${XBotInterface_INCLUDE_DIRS}")
include_directories(${OpenSoT_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS} /opt/xbot/include)
set(SOLVER_LIBRARIES Eigen3::Eigen ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} OpenSoT::OpenSoT XBotInterface::XBotInterface)


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rtt-bullet-embedded
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${OROCOS-RTT_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rtt-bullet-embedded.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rtt-bullet-embedded_node.cpp)

if (NOT OpenSoT_FOUND)
orocos_component(${CMAKE_PROJECT_NAME}
# Kinematics and Dynamics
  ${PROJECT_SOURCE_DIR}/src/kinematics_dynamics/kin_dyn_multi_arm.cpp
  ${PROJECT_SOURCE_DIR}/src/kinematics_dynamics/models/kukaLWRModel.cpp
  ${PROJECT_SOURCE_DIR}/src/kinematics_dynamics/kin_dyn_kdl.cpp

  ${PROJECT_SOURCE_DIR}/src/kinematics_dynamics/rtt_kin_dyn_multi_arm.cpp
# Controller
  ${PROJECT_SOURCE_DIR}/src/controller/gravity_compensation/rtt_joint_gravity_compensation.cpp
  ${PROJECT_SOURCE_DIR}/src/controller/rtt_joint_pd_controller.cpp
  ${PROJECT_SOURCE_DIR}/src/controller/rtt_constant_force_controller.cpp

  # ${PROJECT_SOURCE_DIR}/src/controller/rtt_cart_impedance_controller.cpp
  ${PROJECT_SOURCE_DIR}/src/controller/rtt_cart_pid_controller.cpp
# Prioritization
  ${PROJECT_SOURCE_DIR}/src/prioritization/rtt_dynamic_task_prioritization.cpp
  ${PROJECT_SOURCE_DIR}/src/prioritization/rtt_ghc_projections.cpp
  ${PROJECT_SOURCE_DIR}/src/prioritization/rtt_robot_contact_situations.cpp
# Task
  ${PROJECT_SOURCE_DIR}/src/task/vm_geometry_box.cpp
  ${PROJECT_SOURCE_DIR}/src/task/vm_geometry_bar.cpp
  ${PROJECT_SOURCE_DIR}/src/task/vm_container.cpp
  ${PROJECT_SOURCE_DIR}/src/task/robot_container.cpp
  ${PROJECT_SOURCE_DIR}/src/task/control_objective_container.cpp
  ${PROJECT_SOURCE_DIR}/src/task/control_component_container.cpp
  ${PROJECT_SOURCE_DIR}/src/task/rtt_task_describer_synthesis.cpp

  ${PROJECT_SOURCE_DIR}/src/trajectories/rtt_pose_interpolator.cpp
  # ${PROJECT_SOURCE_DIR}/src/trajectories/rtt_bezier_trajectory.cpp
  ${PROJECT_SOURCE_DIR}/src/trajectories/rtt_linear_trajectory.cpp

  ${PROJECT_SOURCE_DIR}/src/coordination/rtt_contact_situation_switcher.cpp

  ${PROJECT_SOURCE_DIR}/src/introspection/rtt-introspection-base.cpp
  ${PROJECT_SOURCE_DIR}/src/introspection/rtt-call-sample-collector.cpp

# Core Scheduler
  ${PROJECT_SOURCE_DIR}/src/scheduling/rtt-core-scheduler.cpp
  ${PROJECT_SOURCE_DIR}/src/scheduling/barrier-data.cpp
  ${PROJECT_SOURCE_DIR}/src/scheduling/barrier-condition.cpp

) # ...you may add multiple source files
endif()

if (OpenSoT_FOUND)
orocos_component(${CMAKE_PROJECT_NAME}
# Kinematics and Dynamics
  ${PROJECT_SOURCE_DIR}/src/kinematics_dynamics/kin_dyn_multi_arm.cpp
  ${PROJECT_SOURCE_DIR}/src/kinematics_dynamics/models/kukaLWRModel.cpp
  ${PROJECT_SOURCE_DIR}/src/kinematics_dynamics/kin_dyn_kdl.cpp

  ${PROJECT_SOURCE_DIR}/src/kinematics_dynamics/rtt_kin_dyn_multi_arm.cpp
# Controller
  ${PROJECT_SOURCE_DIR}/src/controller/gravity_compensation/rtt_joint_gravity_compensation.cpp
  ${PROJECT_SOURCE_DIR}/src/controller/rtt_joint_pd_controller.cpp
  ${PROJECT_SOURCE_DIR}/src/controller/rtt_constant_force_controller.cpp

  # ${PROJECT_SOURCE_DIR}/src/controller/rtt_cart_impedance_controller.cpp
  ${PROJECT_SOURCE_DIR}/src/controller/rtt_cart_pid_controller.cpp
# Prioritization
  ${PROJECT_SOURCE_DIR}/src/prioritization/rtt_dynamic_task_prioritization.cpp
  ${PROJECT_SOURCE_DIR}/src/prioritization/rtt_ghc_projections.cpp
  ${PROJECT_SOURCE_DIR}/src/prioritization/rtt_robot_contact_situations.cpp
# Task
  ${PROJECT_SOURCE_DIR}/src/task/vm_geometry_box.cpp
  ${PROJECT_SOURCE_DIR}/src/task/vm_geometry_bar.cpp
  ${PROJECT_SOURCE_DIR}/src/task/vm_container.cpp
  ${PROJECT_SOURCE_DIR}/src/task/robot_container.cpp
  ${PROJECT_SOURCE_DIR}/src/task/control_objective_container.cpp
  ${PROJECT_SOURCE_DIR}/src/task/control_component_container.cpp
  ${PROJECT_SOURCE_DIR}/src/task/rtt_task_describer_synthesis.cpp

  ${PROJECT_SOURCE_DIR}/src/trajectories/rtt_pose_interpolator.cpp
  # ${PROJECT_SOURCE_DIR}/src/trajectories/rtt_bezier_trajectory.cpp
  ${PROJECT_SOURCE_DIR}/src/trajectories/rtt_linear_trajectory.cpp

  ${PROJECT_SOURCE_DIR}/src/coordination/rtt_contact_situation_switcher.cpp
  ${PROJECT_SOURCE_DIR}/src/controller/rtt_control_stack.cpp

  ${PROJECT_SOURCE_DIR}/src/introspection/rtt-introspection-base.cpp
  ${PROJECT_SOURCE_DIR}/src/introspection/rtt-call-sample-collector.cpp

# Core Scheduler
  ${PROJECT_SOURCE_DIR}/src/scheduling/rtt-core-scheduler.cpp
  ${PROJECT_SOURCE_DIR}/src/scheduling/barrier-data.cpp
  ${PROJECT_SOURCE_DIR}/src/scheduling/barrier-condition.cpp


) # ...you may add multiple source files
endif()


# set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(${CMAKE_PROJECT_NAME} ${EIGEN3_LIBRARIES} ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${YAML_CPP_LIBRARIES}
${catkin_LIBRARIES}
${SOLVER_LIBRARIES}
/opt/xbot/lib
)

orocos_service(${PROJECT_NAME}-service
  ${PROJECT_SOURCE_DIR}/src/scheduling/core-scheduler-service.cpp
)

target_link_libraries(${PROJECT_NAME}-service
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    #${RTT-CORE-EXTENSIONS_LIBRARIES}
    #${RST-RT_LIBRARIES}
    # NOT SURE IF THIS IS A GOOD IDEA :D
    #${PROJECT_NAME}
)

orocos_generate_package(
  #INCLUDE_DIRS include
  DEPENDS rtt_ros orocos_kdl
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rtt-bullet-embedded.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
