#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>

#include <visualization_msgs/Marker.h>

#include <boost/foreach.hpp>

#include <tf/tf.h>

#include <string.h>
#include <iostream>

#define foreach BOOST_FOREACH


void pub_recorded_marker_old(ros::Publisher &marker_pub, visualization_msgs::Marker::ConstPtr &rosbag_marker, 
                         int index, int total_markers, tf::Transform &transform_to_main ){

    tf::Vector3 marker_position (rosbag_marker->pose.position.x,
                                 rosbag_marker->pose.position.y,
                                 rosbag_marker->pose.position.z);
    tf::Quaternion marker_orientation (rosbag_marker->pose.orientation.x,
                                       rosbag_marker->pose.orientation.y,
                                       rosbag_marker->pose.orientation.z,
                                       rosbag_marker->pose.orientation.w);


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
    marker.pose.position.x = rosbag_marker->pose.position.x;//0;
    marker.pose.position.y = rosbag_marker->pose.position.y;//0;
    marker.pose.position.z = rosbag_marker->pose.position.z;//0;
    marker.pose.orientation.x = rosbag_marker->pose.orientation.x;//0.0;
    marker.pose.orientation.y = rosbag_marker->pose.orientation.y;//0.0;
    marker.pose.orientation.z = rosbag_marker->pose.orientation.z;//0.0;
    marker.pose.orientation.w = rosbag_marker->pose.orientation.w; //1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = rosbag_marker->scale.x; //1.0;
    marker.scale.y = rosbag_marker->scale.y; //1.0;
    marker.scale.z = rosbag_marker->scale.z; //0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0 *  ( (double)(total_markers - index) / (double)total_markers);//rosbag_marker->color.r; //0.0f;
    marker.color.g = 1.0f * ( (double)index / (double)total_markers); //1.0f;//rosbag_marker->color.g; //1.0f;
    marker.color.b = 0.0f;//rosbag_marker->color.b; //0.0f;
    marker.color.a = 1.0f;//1.0 * ( (double)(total_markers - index) / (double)total_markers); //rosbag_marker->color.a; //1.0;

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

}


void pub_recorded_marker(ros::Publisher &marker_pub, visualization_msgs::Marker::ConstPtr &rosbag_marker, 
                         int index, int total_markers, tf::Transform &translate_to_main, tf::Transform &rotate_to_main, int id ){

    tf::Vector3 marker_position (rosbag_marker->pose.position.x,
                                 rosbag_marker->pose.position.y,
                                 rosbag_marker->pose.position.z);
    tf::Quaternion marker_orientation (rosbag_marker->pose.orientation.x,
                                       rosbag_marker->pose.orientation.y,
                                       rosbag_marker->pose.orientation.z,
                                       rosbag_marker->pose.orientation.w);


    //marker_position = marker_position - offset_vector; //transform_to_main * 
    marker_position = translate_to_main * marker_position; 
    marker_orientation = rotate_to_main * marker_orientation;

     visualization_msgs::Marker marker;
//    uint32_t shape = visualization_msgs::Marker::CUBE;
//    uint32_t shape = visualization_msgs::Marker::SPHERE;
        uint32_t shape = visualization_msgs::Marker::ARROW;
     if (id == 6){
        shape = visualization_msgs::Marker::CUBE;        
    }
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //marker.header.frame_id = "/my_frame";
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = rosbag_marker->ns;//"basic_shapes";
    
    marker.id = index;//rosbag_marker->id;

     if (id == 6){
        marker.id = index+500;        
    }

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape; //rosbag_marker->type; 

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    //Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // marker.pose.position.x = marker_position.getX();//rosbag_marker->pose.position.x;//0;
    // marker.pose.position.y = marker_position.getY();//rosbag_marker->pose.position.y;//0;
    // marker.pose.position.z = marker_position.getZ();//rosbag_marker->pose.position.z;//0;
    // marker.pose.orientation.x = marker_orientation.getAxis().getX();//rosbag_marker->pose.orientation.x;//0.0;
    // marker.pose.orientation.y = marker_orientation.getAxis().getY();//rosbag_marker->pose.orientation.y;//0.0;
    // marker.pose.orientation.z = marker_orientation.getAxis().getZ();//rosbag_marker->pose.orientation.z;//0.0;
    // marker.pose.orientation.w = marker_orientation.getW();//rosbag_marker->pose.orientation.w; //1.0;


    // Try some coordinate transformations:
    // marker.pose.position.z = marker_position.getX();//rosbag_marker->pose.position.x;//0;
    // marker.pose.position.y = marker_position.getY();//rosbag_marker->pose.position.y;//0;
    // marker.pose.position.x = -marker_position.getZ();//rosbag_marker->pose.position.z;//0;


    marker.pose.position.x = -marker_position.getZ();//rosbag_marker->pose.position.x;//0;
    marker.pose.position.y = marker_position.getX();//rosbag_marker->pose.position.y;//0;
    marker.pose.position.z = -marker_position.getY();//rosbag_marker->pose.position.z;//0;


    marker.pose.orientation.x = 1;//rosbag_marker->pose.orientation.x;//0.0;
    marker.pose.orientation.y = 0;//rosbag_marker->pose.orientation.y;//0.0;
    marker.pose.orientation.z = 0;//rosbag_marker->pose.orientation.z;//0.0;
    marker.pose.orientation.w = 0;//rosbag_marker->pose.orientation.w; //1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.02;//rosbag_marker->scale.x; //1.0;
    marker.scale.y = 0.005;//rosbag_marker->scale.y; //1.0;
    marker.scale.z = 0.005;//rosbag_marker->scale.z; //0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0 *  ( (double)(total_markers - index) / (double)total_markers);//rosbag_marker->color.r; //0.0f;
    marker.color.g = 1.0f * ( (double)index / (double)total_markers); //1.0f;//rosbag_marker->color.g; //1.0f;
    marker.color.b = 0.0f;//rosbag_marker->color.b; //0.0f;
    marker.color.a = 1.0f;//1.0 * ( (double)(total_markers - index) / (double)total_markers); //rosbag_marker->color.a; //1.0;
    if (id == 6){
        marker.pose.orientation.x = 1;//rosbag_marker->pose.orientation.x;//0.0;
        marker.pose.orientation.y = 0;//rosbag_marker->pose.orientation.y;//0.0;
        marker.pose.orientation.z = 0;//rosbag_marker->pose.orientation.z;//0.0;
        marker.pose.orientation.w = 0;//rosbag_marker->pose.orientation.w; //1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1;//rosbag_marker->scale.x; //1.0;
        marker.scale.y = 0.1;//rosbag_marker->scale.y; //1.0;
        marker.scale.z = 0.1;//rosbag_marker->scale.z; //0.5;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0; //*  ( (double)(total_markers - index) / (double)total_markers);//rosbag_marker->color.r; //0.0f;
    marker.color.g = 0.0;//f * ( (double)index / (double)total_markers); //1.0f;//rosbag_marker->color.g; //1.0f;
    marker.color.b = 1.0;//rosbag_marker->color.b; //0.0f;
    marker.color.a = 0.02f;//1.0 * ( (double)(total_markers - index) / (double)total_markers); //rosbag_marker->color.a; //1.0;        
    }


    

 

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
    //sleep(1.0);
}


int main(int argc, char **argv){
    ros::init (argc, argv, "viz_rosbag");
    ros::NodeHandle n;
    ros::Publisher rvizMarkerPub; 
    rvizMarkerPub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1000);

    ROS_INFO("Opening Bag");
    rosbag::Bag bag;    
    bag.open("bag_files/LR_lowerWave.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/visualization_marker")); //Specify topic to read
    rosbag::View view(bag, rosbag::TopicQuery(topics));

   // Define Reference quaternion to be the x-axis.
    tf::Quaternion main_axis(1,0,0, 1);
    main_axis = main_axis.normalize();

    // Define the position and orientation of the first marker
    tf::Vector3 first_marker_vector_offset;
    tf::Quaternion first_marker_axis;

    // Define Quaternion Rotation Offset
    tf::Quaternion axis_rotation;

    
    int marker_index = 0;
    int marker_id_to_viz = 1; // 4 for the hand traj. 6 for the obj traj.

    // Count total number of markers
    // Identify the first marker and use its position and orientation to identify the first rotation
    int total_markers = 0;
    foreach(rosbag::MessageInstance const m, view){
        visualization_msgs::Marker::ConstPtr first_marker = m.instantiate<visualization_msgs::Marker>();
        if (first_marker-> id == marker_id_to_viz){
            if (total_markers == 0){
    //            visualization_msgs::Marker::ConstPtr first_marker = m.instantiate<visualization_msgs::Marker>();

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
    }        

              axis_rotation = first_marker_axis.inverse() * main_axis; // Find axis of rotation
              tf::Transform transform_to_main_axis(tf::Quaternion(0,0,0,1), -first_marker_vector_offset); // Create transform
              tf::Transform rotate_to_main_axis(axis_rotation, tf::Vector3(0,0,0)); // Create transform
              //tf::Transform transform_to_main_axis(axis_rotation, -first_marker_vector_offset); // Create transform

//            ROS_INFO("non-zero");
//            std::cout << first_marker_vector_offset.getZ() << std::endl;
//            sleep(1);

    foreach(rosbag::MessageInstance const m, view){
        //std::cout << m.getTopic() << std::endl;
        ROS_INFO("Inside bag!");
        //geometry_msgs::Pose::ConstPtr p = m.instantiate<geometry_msgs::Pose>();
        visualization_msgs::Marker::ConstPtr p = m.instantiate<visualization_msgs::Marker>();

            std::cout << m.getDataType() << std::endl; // Identify topic type
            std::cout << p->id << std::endl;
            std::cout << p->pose.position.x << std::endl;
            std::cout << p->pose.position.y << std::endl;
            std::cout << p->pose.position.z << std::endl;
            std::cout << p->pose.orientation.x << std::endl;
            std::cout << p->pose.orientation.y << std::endl;
            std::cout << p->pose.orientation.z << std::endl;
            std::cout << p->pose.orientation.w << std::endl;

            pub_recorded_marker(rvizMarkerPub, p, marker_index, total_markers, transform_to_main_axis, rotate_to_main_axis, p->id);
       if (p-> id == marker_id_to_viz){            
            marker_index++;
       }
        //sleep(1.0);
        std::cout << total_markers << std::endl;
    }
    ROS_INFO("Closing bag");

    bag.close();

}