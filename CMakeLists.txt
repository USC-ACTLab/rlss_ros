cmake_minimum_required(VERSION 2.8.3)
project(rlss_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin REQUIRED COMPONENTS
        roscpp
        message_generation
#        create2_controller
#        nav_msgs
        std_msgs
        tf
        geometry_msgs
        genmsg
        actionlib_msgs
        actionlib
#        crazyflie_driver
)

find_package(
    Boost REQUIRED COMPONENTS filesystem
)

add_subdirectory(third_party/rlss)

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
add_message_files(
    FILES
    Bezier.msg
    PiecewiseTrajectory.msg
    AABB.msg
    RobotState.msg
    AABBCollisionShapeAndState.msg
    OccupancyGrid.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   DesiredTrajectory.srv
   SetOnOff.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY
   action
   FILES
   FollowTrajectory.action
   Land.action
   Takeoff.action
   NAV.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    actionlib_msgs
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
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rlss-ros
  CATKIN_DEPENDS message_runtime
#  DEPENDS Boost
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
#   src/${PROJECT_NAME}/rlss-ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rlss-ros_node.cpp)

#add_executable(
#        figure8
#        src/figure8.cpp
#)
#target_link_libraries(
#        figure8
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        figure8
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#
#add_executable(
#        figure8_3d
#        src/figure8_3d.cpp
#)
#target_link_libraries(
#        figure8_3d
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        figure8_3d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#
add_executable(
        replanner_3d
        src/replanner.cpp
)
target_compile_definitions(
        replanner_3d PUBLIC
        DIMENSION=3
)
target_link_libraries(
        replanner_3d
        ${catkin_LIBRARIES}
        rlss
)
add_dependencies(
        replanner_3d
        ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(
        follow_trajectory_action_server_3d
        src/follow_trajectory_action_server.cpp
)
target_compile_definitions(
        follow_trajectory_action_server_3d PUBLIC
        DIMENSION=3
)
target_link_libraries(
        follow_trajectory_action_server_3d
        ${catkin_LIBRARIES}
        rlss
)
add_dependencies(
        follow_trajectory_action_server_3d
        ${rlss_ros_EXPORTED_TARGETS}
)


add_executable(
        land_action_server_3d
        src/land_action_server.cpp
)
target_compile_definitions(
        land_action_server_3d PUBLIC
        DIMENSION=3
)
target_link_libraries(
        land_action_server_3d
        ${catkin_LIBRARIES}
        rlss
)
add_dependencies(
        land_action_server_3d
        ${rlss_ros_EXPORTED_TARGETS}
)


add_executable(
        takeoff_action_server_3d
        src/takeoff_action_server.cpp
)
target_compile_definitions(
        takeoff_action_server_3d PUBLIC
        DIMENSION=3
)
target_link_libraries(
        takeoff_action_server_3d
        ${catkin_LIBRARIES}
        rlss
)
add_dependencies(
        takeoff_action_server_3d
        ${rlss_ros_EXPORTED_TARGETS}
)


add_executable(
        nav_action_server_3d
        src/nav_action_server.cpp
)
target_compile_definitions(
        nav_action_server_3d PUBLIC
        DIMENSION=3
)
target_link_libraries(
        nav_action_server_3d
        ${catkin_LIBRARIES}
        rlss
)
add_dependencies(
        nav_action_server_3d
        ${rlss_ros_EXPORTED_TARGETS}
)
#
#add_executable(
#        static_occupancy_grid_feeder_3d
#        src/static_occupancy_grid_feeder.cpp
#)
#target_compile_definitions(
#        static_occupancy_grid_feeder_3d PUBLIC
#        DIMENSION=3
#)
#target_link_libraries(
#        static_occupancy_grid_feeder_3d
#        ${catkin_LIBRARIES}
#        rlss
#        ${Boost_LIBRARIES}
#        ${Boost_FILESYSTEM_LIBRARY}
#        ${Boost_SYSTEM_LIBRARY}
#)
#add_dependencies(
#        static_occupancy_grid_feeder_3d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#target_include_directories(
#        static_occupancy_grid_feeder_3d PUBLIC
#        ${Boost_INCLUDE_DIRS}
#)

#
#add_executable(
#        planner_2d
#        src/planner.cpp
#)
#target_compile_definitions(
#        planner_2d PUBLIC
#        DIMENSION=2
#)
#target_link_libraries(
#        planner_2d
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        planner_2d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#add_executable(
#        static_occupancy_grid_feeder_2d
#        src/static_occupancy_grid_feeder.cpp
#)
#target_compile_definitions(
#        static_occupancy_grid_feeder_2d PUBLIC
#        DIMENSION=2
#)
#target_link_libraries(
#        static_occupancy_grid_feeder_2d
#        ${catkin_LIBRARIES}
#        rlss
#        ${Boost_LIBRARIES}
#        ${Boost_FILESYSTEM_LIBRARY}
#        ${Boost_SYSTEM_LIBRARY}
#)
#add_dependencies(
#        static_occupancy_grid_feeder_2d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#target_include_directories(
#        static_occupancy_grid_feeder_3d PUBLIC
#        ${Boost_INCLUDE_DIRS}
#)
#
#add_executable(
#        commander_2d
#        src/commander.cpp
#)
#target_link_libraries(
#        commander_2d
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        commander_2d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#target_compile_definitions(
#        commander_2d PUBLIC
#        DIMENSION=2
#)
#
add_executable(
        commander_3d
        src/commander.cpp
)
target_link_libraries(
        commander_3d
        ${catkin_LIBRARIES}
        rlss
)
add_dependencies(
        commander_3d
        ${PROJECT_NAME}_generate_messages_cpp
)
target_compile_definitions(
        commander_3d PUBLIC
        DIMENSION=3
)
#
#add_executable(
#        create2_state_reducer
#        src/create2/state_reducer.cpp
#)
#target_link_libraries(
#        create2_state_reducer
#        ${catkin_LIBRARIES}
#)
#add_dependencies(
#        create2_state_reducer
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#
#add_executable(
#        create2_localizer_2d
#        src/create2/localizer.cpp
#)
#target_link_libraries(
#        create2_localizer_2d
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        create2_localizer_2d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#target_compile_definitions(
#        create2_localizer_2d PUBLIC
#        DIMENSION=2
#)
#
#add_executable(
#        create2_localizer_3d
#        src/create2/localizer.cpp
#)
#target_link_libraries(
#        create2_localizer_3d
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        create2_localizer_3d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#target_compile_definitions(
#        create2_localizer_3d PUBLIC
#        DIMENSION=3
#)
#
#add_executable(
#        static_desired_trajectory_feeder_2d
#        src/static_desired_trajectory_feeder.cpp
#)
#target_link_libraries(
#        static_desired_trajectory_feeder_2d
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        static_desired_trajectory_feeder_2d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#target_compile_definitions(
#        static_desired_trajectory_feeder_2d PUBLIC
#        DIMENSION=2
#)
#
#add_executable(
#        static_desired_trajectory_feeder_3d
#        src/static_desired_trajectory_feeder.cpp
#)
#target_link_libraries(
#        static_desired_trajectory_feeder_3d
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        static_desired_trajectory_feeder_3d
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#target_compile_definitions(
#        static_desired_trajectory_feeder_3d PUBLIC
#        DIMENSION=3
#)

add_executable(
        tf_localizer_3d
        src/tf_localizer.cpp
)
target_link_libraries(
        tf_localizer_3d
        ${catkin_LIBRARIES}
        rlss
)
add_dependencies(
        tf_localizer_3d
        ${PROJECT_NAME}_generate_message_cpp
)
target_compile_definitions(
        tf_localizer_3d PUBLIC
        DIMENSION=3
)
add_executable(
        hector_quadrotor_controller
        src/hector_quadrotor/controller.cpp
)
target_link_libraries(
        hector_quadrotor_controller
        ${catkin_LIBRARIES}
        rlss
)
add_dependencies(
        hector_quadrotor_controller
        ${PROJECT_NAME}_generate_message_cpp
)

add_executable(
        cf_desired_state_reducer
        src/cf/state_reducer.cpp
)
target_link_libraries(
        cf_desired_state_reducer
        ${catkin_LIBRARIES}
)
add_dependencies(
        cf_desired_state_reducer
        ${PROJECT_NAME}_generate_messages_cpp
)

#
#add_executable(
#        cf2_state_reducer
#        src/cf2/state_reducer.cpp
#)
#target_link_libraries(
#        cf2_state_reducer
#        ${catkin_LIBRARIES}
#)
#add_dependencies(
#        cf2_state_reducer
#        ${PROJECT_NAME}_generate_messages_cpp
#)
#
#add_executable(
#        cf2_localizer
#        src/cf2/localizer.cpp
#)
#target_link_libraries(
#        cf2_localizer
#        ${catkin_LIBRARIES}
#        rlss
#)
#add_dependencies(
#        cf2_localizer
#        ${PROJECT_NAME}_generate_message_cpp
#)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rlss-ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
