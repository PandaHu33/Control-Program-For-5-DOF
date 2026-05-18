cmake_minimum_required(VERSION 3.0.2)
project(mainpulator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  joy
  sensor_msgs
  geometry_msgs
)
#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${qpOASES_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})



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
  INCLUDE_DIRS 
    include
  LIBRARIES 
    control
    param
    trajectory
    
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
include_directories("../mainpulator/include/mavlink/common")#引入mavlink的库
include_directories("../mainpulator/include/mavlink/ardupilotmega")#引入mavlink的库
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mainpulator.cpp
# )
add_library(mainpulator_library
  include/mainpulator/mainpulator_control.h
  src/mainpulator_control.cpp

  include/mainpulator/mainpulator_trajectory.h
  src/mainpulator_trajectory.cpp

  include/mainpulator/mainpulator_param.h
  src/mainpulator_param.cpp

  include/mainpulator/MyTcpip.h

  include/mainpulator/MyRoboticKinematics.h
  src/MyRoboticKinematics.cpp

  include/mainpulator/PipeFilterExpansion.h

)
target_link_libraries(mainpulator_library
 ${catkin_LIBRARIES}
)




add_dependencies(mainpulator_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mainpulator_node.cpp)

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

add_executable(test_node src/test_node.cpp)
# add_executable(Z_demo01_pub src/Z_demo01_pub.cpp)
# add_executable(Z_demo02_sub src/Z_demo02_sub.cpp)


#add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_node
   mainpulator_library
   ${catkin_LIBRARIES}
   qpOASES
)
# target_link_libraries(Z_demo01_pub 
#     ${catkin_LIBRARIES}
# )
# target_link_libraries(Z_demo02_sub 
#     ${catkin_LIBRARIES}
# )




add_executable(trajectory_node src/trajectory_node.cpp)

#add_dependencies(trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_node
    mainpulator_library
    ${catkin_LIBRARIES}
)

add_executable(camera_node src/camera_node.cpp)

#add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_node
    mainpulator_library
    ${catkin_LIBRARIES}
)

add_executable(stop_node src/stop_node.cpp)

#add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stop_node
    mainpulator_library
    ${catkin_LIBRARIES}
)
add_executable(tcp_receive_node src/tcp_receive_node.cpp)
target_link_libraries(tcp_receive_node
    mainpulator_library
    ${catkin_LIBRARIES}
)


add_executable(mainpulator_stop_node src/mainpulator_stop.cpp)
target_link_libraries(mainpulator_stop_node
    mainpulator_library
    ${catkin_LIBRARIES}
)


add_executable(teleop_keyboard_node src/teleop_keyboard.cpp)
target_link_libraries(teleop_keyboard_node
    mainpulator_library
    ${catkin_LIBRARIES}
)

add_executable(teleop_joy_node src/teleop_joy.cpp)
target_link_libraries(teleop_joy_node
    mainpulator_library
    ${catkin_LIBRARIES}
)

add_executable(mavlink_udp_node src/mavlink_udp.cpp)
target_link_libraries(mavlink_udp_node
    mainpulator_library
    ${catkin_LIBRARIES}
)

add_executable(Imu_Euler_Node src/imu_euler_node.cpp)
target_link_libraries(Imu_Euler_Node
    mainpulator_library
    ${catkin_LIBRARIES}
)

add_executable(Imu_111_Node src/imu111.cpp)
target_link_libraries(Imu_111_Node
    mainpulator_library
    ${catkin_LIBRARIES}
)

#add_executable(tag_detector src/tag_detector.cpp)
#target_link_libraries(tag_detector
#    mainpulator_library
#    ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mainpulator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
