cmake_minimum_required(VERSION 2.8.3)
project(try_moveit)

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


add_executable(try_move_pr2_right_arm src/try_move_pr2_right_arm.cpp)
target_link_libraries(try_move_pr2_right_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS try_move_pr2_right_arm DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(moveit_rosbag src/moveit_rosbag.cpp)
target_link_libraries(moveit_rosbag ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS moveit_rosbag DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(rosbag_visualize_marker_trajectory src/rosbag_visualize_marker_trajectory.cpp)
target_link_libraries(rosbag_visualize_marker_trajectory ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS rosbag_visualize_marker_trajectory DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(simple_move_pr2_right_arm src/simple_move_pr2_right_arm.cpp)
target_link_libraries(simple_move_pr2_right_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS simple_move_pr2_right_arm DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_subdirectory(kinematics)
#add_subdirectory(planning)
#add_subdirectory(state_display)
#add_subdirectory(interactivity)
#add_subdirectory(pick_place)
