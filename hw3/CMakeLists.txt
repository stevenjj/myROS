cmake_minimum_required(VERSION 2.8.3)
project(hw3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  pluginlib cmake_modules
  rosbag
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    rosbag
    roscpp
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(circ_trajectory_viz_execution src/circ_trajectory_viz_execution.cpp)
target_link_libraries(circ_trajectory_viz_execution ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS circ_trajectory_viz_execution DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(circ_viz_rosbag src/circ_viz_rosbag.cpp)
target_link_libraries(circ_viz_rosbag ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS circ_viz_rosbag DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(dmp_f_target_calc src/dmp_f_target_calc.cpp)
target_link_libraries(dmp_f_target_calc ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS dmp_f_target_calc DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_subdirectory(kinematics)
#add_subdirectory(planning)
#add_subdirectory(state_display)
#add_subdirectory(interactivity)
#add_subdirectory(pick_place)
