#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>
#include <tf/tf.h>

#include <string.h>
#include <iostream>


#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>



#define foreach BOOST_FOREACH


// Create global to make my life easier
std::vector<geometry_msgs::Pose> waypoints;

// Publish recorded markers but also push waypoint trajectories.
void pub_recorded_marker(ros::Publisher &marker_pub, visualization_msgs::Marker::ConstPtr &rosbag_marker, 
                         int index, int total_markers, tf::Transform &translate_to_main, tf::Transform &rotate_to_main ){

    tf::Vector3 marker_position (rosbag_marker->pose.position.x,
                                 rosbag_marker->pose.position.y,
                                 rosbag_marker->pose.position.z);
    tf::Quaternion marker_orientation (rosbag_marker->pose.orientation.x,
                                       rosbag_marker->pose.orientation.y,
                                       rosbag_marker->pose.orientation.z,
                                       rosbag_marker->pose.orientation.w);

    // Perform tf transformation.
    marker_position = translate_to_main * marker_position; 
    marker_orientation = rotate_to_main * marker_orientation;

     visualization_msgs::Marker marker;
    uint32_t shape = visualization_msgs::Marker::CUBE;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //marker.header.frame_id = "/my_frame";
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = rosbag_marker->ns;//"basic_shapes";
    marker.id = index;//rosbag_marker->id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = rosbag_marker->type; //shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_position.getX();//rosbag_marker->pose.position.x;//0;
    marker.pose.position.y = marker_position.getY();//rosbag_marker->pose.position.y;//0;
    marker.pose.position.z = marker_position.getZ();//rosbag_marker->pose.position.z;//0;
    marker.pose.orientation.x = marker_orientation.getAxis().getX();//rosbag_marker->pose.orientation.x;//0.0;
    marker.pose.orientation.y = marker_orientation.getAxis().getY();//rosbag_marker->pose.orientation.y;//0.0;
    marker.pose.orientation.z = marker_orientation.getAxis().getZ();//rosbag_marker->pose.orientation.z;//0.0;
    marker.pose.orientation.w = marker_orientation.getW();//rosbag_marker->pose.orientation.w; //1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = rosbag_marker->scale.x; //1.0;
    marker.scale.y = rosbag_marker->scale.y; //1.0;
    marker.scale.z = rosbag_marker->scale.z; //0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    // use index/total_markers percentage as a way to fade from red to green.
    marker.color.r = 1.0 *  ( (double)(total_markers - index) / (double)total_markers);//rosbag_marker->color.r; //0.0f;
    marker.color.g = 1.0f * ( (double)index / (double)total_markers); //1.0f;//rosbag_marker->color.g; //1.0f;
    marker.color.b = 0.0f;//rosbag_marker->color.b; //0.0f;
    marker.color.a = 1.0f;//1.0 * ( (double)(total_markers - index) / (double)total_markers); //rosbag_marker->color.a; //1.0;

    // Make it last forever. We don't need to keep publishing it.
    marker.lifetime = ros::Duration();

    // Push marker 6DoF Pose to the waypoints vector.
    geometry_msgs::Pose target_pose;
    target_pose.position.x = marker_position.getX();
    target_pose.position.y = marker_position.getY();
    target_pose.position.z = marker_position.getZ();

    target_pose.orientation.x = marker_orientation.getAxis().getX();//rosbag_marker->pose.orientation.x;//0.0;
    target_pose.orientation.y = marker_orientation.getAxis().getY();//rosbag_marker->pose.orientation.y;//0.0;
    target_pose.orientation.z = marker_orientation.getAxis().getZ();//rosbag_marker->pose.orientation.z;//0.0;
    target_pose.orientation.w = marker_orientation.getW();//rosbag_marker->pose.orientation.w; //1.0;

    waypoints.push_back(target_pose);  

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        ROS_WARN_ONCE("NOT OK!");
        break;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1.0);
    }
    marker_pub.publish(marker);

}


int main(int argc, char **argv){    
  ros::init (argc, argv, "trajectory_viz_execution");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher rvizMarkerPub; 
  rvizMarkerPub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1000);



  moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  //ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  
  // Move the robot to a known starting position
  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.x = 1.0;
  start_pose2.orientation.y = 0.0; 
  start_pose2.orientation.z = 0.0;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;//0.55;
  start_pose2.position.y = -0.55;//-0.05;
  start_pose2.position.z = 0.8;//0.8;

  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);
  
  // Now we will plan to the earlier pose target from the new 
  // start state that we have just created.
  group.setPoseTarget(start_pose2);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Moving to start position %s",success?"":"FAILED");
  group.move();
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(10.0);
  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();


    ROS_INFO("Opening Bag");
    rosbag::Bag bag;    


    bag.open("/home/stevenjj/catkin_ws/src/try_moveit/trajectory_bag.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("visualization_marker")); //Specify topic to read
    rosbag::View view(bag, rosbag::TopicQuery(topics));

   // Define Reference quaternion to be the x-axis.
    //tf::Quaternion main_axis(1,0,0, 1);
    tf::Vector3 r_gripper_position(start_pose2.position.x, start_pose2.position.y, start_pose2.position.z);    
    tf::Quaternion main_axis(start_pose2.orientation.x, start_pose2.orientation.y, start_pose2.orientation.z, start_pose2.orientation.w);
    main_axis = main_axis.normalize();

    // Define the position and orientation of the first marker
    tf::Vector3 first_marker_vector_offset;
    tf::Quaternion first_marker_axis;

    // Define Quaternion Rotation Offset    
    tf::Quaternion axis_rotation;

    
    int marker_index = 0;

    // Count total number of markers
    // Identify the first marker and use its position and orientation to identify the first rotation
    int total_markers = 0;
    foreach(rosbag::MessageInstance const m, view){
        if (total_markers == 0){
            visualization_msgs::Marker::ConstPtr first_marker = m.instantiate<visualization_msgs::Marker>();

            first_marker_vector_offset.setX(first_marker->pose.position.x);
            first_marker_vector_offset.setY(first_marker->pose.position.y);
            first_marker_vector_offset.setZ(first_marker->pose.position.z);              

            tf::Quaternion store_axis (first_marker->pose.orientation.x, 
                                                  first_marker->pose.orientation.y, 
                                                  first_marker->pose.orientation.z,
                                                  first_marker->pose.orientation.w);
            first_marker_axis = store_axis;
         }
        total_markers++; // count total number of markers in the rosbag
    }        

    // Fancy Quaternion math. This took forever. p' = p_a^-1 * p2 gives the rotation needed to go from p_a to p2.
    axis_rotation = first_marker_axis.inverse() * main_axis; // Find axis of rotation
    tf::Transform transform_to_main_axis(tf::Quaternion(0,0,0,1), r_gripper_position - first_marker_vector_offset); // Create transform
    tf::Transform rotate_to_main_axis(axis_rotation, tf::Vector3(0,0,0)); // Create transform


    foreach(rosbag::MessageInstance const m, view){
        //std::cout << m.getTopic() << std::endl;
        ROS_INFO("Inside bag!");
        //geometry_msgs::Pose::ConstPtr p = m.instantiate<geometry_msgs::Pose>();
        visualization_msgs::Marker::ConstPtr p = m.instantiate<visualization_msgs::Marker>();
        //std::cout << m.getDataType() << std::endl; // Identify topic type
        //std::cout << p->pose.position.x << std::endl;

        pub_recorded_marker(rvizMarkerPub, p, marker_index, total_markers, transform_to_main_axis, rotate_to_main_axis);
        marker_index++;
        //sleep(1.0);
        std::cout << total_markers << std::endl;
    }
    ROS_INFO("Closing bag");

    bag.close();



  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints 
  // for the end-effector to go through. Note that we are starting 
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list.

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);    
  /* Sleep to give Rviz time to visualize the plan. */

  ROS_INFO("Executing plan 4 (cartesian path)");

  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory;
  group.execute(plan);
  
  sleep(15.0);
  ros::shutdown();  




}