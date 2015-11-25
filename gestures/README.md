
To make python executable
chmod +x myscript.py

To record data using the kinect and the ar tracker

roscore
roslaunch openni_launch openni.launch 
rosrun ar_track_alvar individualMarkers 4.5 0.08 0.2 /camera/depth_registered/points /camera/rgb/camera_info /camera_link

rosrun rviz rviz

Then load visualize_markers.rviz

navigate to catkin_ws/src/gesture stack then:

rosrun gestures dmp_calculate_f_target 'bag_files/LL_to_UR28.bag' > text.txt
rosrun gestures dmp_calculate_f_target 'bag_files2/circle_8.bag' > 'bag_files2/txt_files/circle_8.txt'

Change the bag file name and the output text file as desired.

To see the current marker pose:
rostopic echo /visualization_marker 

To record marker pose as a rosbag:
rosbag record /visualization_marker
