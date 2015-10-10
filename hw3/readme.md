Testing DMP
- navigate to /hw3 directory so that .bag files are visible

rosrun hw3 dmp_f_target_calc.cpp

Visualizing trajectory:
- navigate to /hw3 directory so that .bag files are visible

rosrun rviz rviz
rosrun hw3 circ_viz_rosbag.cpp


Demo
launch sequence:

roslaunch pr2_gazebo pr2_empty_world.launch 
rosrun rviz rviz

roslaunch try_moveit pr2_follow_trajectory.launch 