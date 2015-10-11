/-------------------------------------------------------------------------------------------
Testing DMP
- navigate to /hw3 directory so that .bag files are visible

rosrun hw3 dmp_f_target_calc.cpp

Visualizing trajectory:
- navigate to /hw3 directory so that .bag files are visible

rosrun rviz rviz
rosrun hw3 circ_viz_rosbag.cpp
/-------------------------------------------------------------------------------------------

Spawning and Deleting models from command line.

rosrun gazebo_ros spawn_model -file `pwd`/models/table.sdf -sdf -model table_1 -x 0.0 -z 0
rosservice call gazebo/delete_model '{model_name: table_1}'



//
sudo apt-get install ros-hydro-openni-launch ros-hydro-openni-tracker ros-hydro-openni-camera

Running Kinnect
roslaunch openni_launch openni.launch 

Run rviz
rosrun rviz rviz

Running AR Tracking
rosrun ar_track_alvar individualMarkers 4.5 0.08 0.2 /camera/depth_registered/points /camera/rgb/camera_info /camera_link

rosbag record visualization_marker
//-------------------------------------------------


rosrun tf static_transform_publisher 1 0 0 .1 0 0 0 /torso_lift_link /camera_link 50

rosrun tf static_transform_publisher 1 0 0 .1 0 0 0 /base_link /camera_link 50

rosrun tf static_transform_publisher 1 0 1 0 0 0 /base_link /camera_link 50



Demo
launch sequence:

roslaunch pr2_gazebo pr2_empty_world.launch 
rosrun rviz rviz

roslaunch hw3 pr2_dmp_execute.launch 




rosrun gazebo_ros gazebo