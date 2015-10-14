#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>

#include <string.h>
#include <iostream>
#include <math.h>

#include <gazebo_msgs/GetModelState.h>

void pub_object_marker(ros::Publisher &marker_pub, ros::NodeHandle &n){
     visualization_msgs::Marker marker;
//    uint32_t shape = visualization_msgs::Marker::CUBE;
//    uint32_t shape = visualization_msgs::Marker::SPHERE;
    uint32_t shape = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //marker.header.frame_id = "/my_frame";
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 100;//rosbag_marker->id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape; //rosbag_marker->type; 

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;




    ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getmodelstate;

    getmodelstate.request.model_name = "block";
    gms_c.call(getmodelstate);

    double x = getmodelstate.response.pose.position.x;
    double y = getmodelstate.response.pose.position.y;
    double z = getmodelstate.response.pose.position.z;

    double q_x = getmodelstate.response.pose.orientation.x;
    double q_y = getmodelstate.response.pose.orientation.y;
    double q_z = getmodelstate.response.pose.orientation.z;
    double q_w = getmodelstate.response.pose.orientation.w;

    double z_offset = 0.000575;
    std::cout << x << std::endl;
    std::cout << y << std::endl;
    std::cout << z - z_offset << std::endl;    

    marker.pose.position.x = x;//rosbag_marker->pose.position.x;//0;
    marker.pose.position.y = y;//rosbag_marker->pose.position.y;//0;
    marker.pose.position.z = z - z_offset;//rosbag_marker->pose.position.z;//0;
    marker.pose.orientation.x = q_x;//rosbag_marker->pose.orientation.x;//0.0;
    marker.pose.orientation.y = q_y;//rosbag_marker->pose.orientation.y;//0.0;
    marker.pose.orientation.z = q_z;//rosbag_marker->pose.orientation.z;//0.0;
    marker.pose.orientation.w = q_w;//rosbag_marker->pose.orientation.w; //1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;//rosbag_marker->scale.x; //1.0;
    marker.scale.y = 0.165;//rosbag_marker->scale.y; //1.0;
    marker.scale.z = 0.1;//rosbag_marker->scale.z; //0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0 ; // *  ( (double)(total_markers - index) / (double)total_markers);//rosbag_marker->color.r; //0.0f;
    marker.color.g = 0.0f; // * ( (double)index / (double)total_markers); //1.0f;//rosbag_marker->color.g; //1.0f;
    marker.color.b = 1.0f;//rosbag_marker->color.b; //0.0f;
    marker.color.a = 0.75f;//1.0 * ( (double)(total_markers - index) / (double)total_markers); //rosbag_marker->color.a; //1.0;

    marker.lifetime = ros::Duration();

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
    // std::cout << "success?" << std::endl;
}

int main(int argc, char **argv){
    ros::init (argc, argv, "block_marker");
    ros::NodeHandle n;
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    ros::Rate r(1);
    ros::Publisher rvizMarkerPub; // Create Marker Publisher
    rvizMarkerPub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 10); // Advertise Marker Publisher

	while (ros::ok()){
		pub_object_marker(rvizMarkerPub,n);		
		sleep(1.0);
//		r.sleep();	
	}

}