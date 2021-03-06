#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>
#include <tf/tf.h>

//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>

#include <string.h>
#include <iostream>
#include <math.h>

#define foreach BOOST_FOREACH

void pub_recorded_marker(ros::Publisher &marker_pub, visualization_msgs::Marker::ConstPtr &rosbag_marker, 
                         int index, int total_markers, tf::Transform &translate_to_main, tf::Transform &rotate_to_main ){

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

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.my_frame
    //marker.header.frame_id = "/my_frame";
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = rosbag_marker->ns;//"basic_shapes";
    marker.id = index;//rosbag_marker->id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape; //rosbag_marker->type; 

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Try some coordinate transformations:
    marker.pose.position.z = marker_position.getX();//rosbag_marker->pose.position.x;//0;
    marker.pose.position.y = marker_position.getY();//rosbag_marker->pose.position.y;//0;
    marker.pose.position.x = -marker_position.getZ();//rosbag_marker->pose.position.z;//0;
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







// METHOD 1 of velocity and acceleration
// Calculates constant acceleration between two points x0, x1 and factors in velocity between them.
double calc_acel(double x1, double xo, double vo, double dt){
    return ( (x1 - xo - (vo*dt))/pow(dt,2) );
}

// Returns acceleration for x,y,z
tf::Vector3 constant_a(tf::Vector3 &pos_next_i, tf::Vector3 &pos_i, tf::Vector3 &vel_i, double dt){
    double a_x = calc_acel( pos_next_i.getX(), pos_i.getX(), vel_i.getX(), dt);
    double a_y = calc_acel( pos_next_i.getY(), pos_i.getY(), vel_i.getY(), dt);
    double a_z = calc_acel( pos_next_i.getZ(), pos_i.getZ(), vel_i.getZ(), dt);
    return tf::Vector3(a_x,a_y,a_z);
}

// Calculates velocity(i+1) from acceleration and prevoius velocity
double calc_vel(double a_o, double v_o, double dt){
    return a_o*dt + v_o;
}

tf::Vector3 vel_from_a(tf::Vector3 &acel_i, tf::Vector3 &vel_i, double dt){
    double v_x = calc_vel( acel_i.getX(), vel_i.getX(), dt);
    double v_y = calc_vel( acel_i.getY(), vel_i.getY(), dt);
    double v_z = calc_vel( acel_i.getZ(), vel_i.getZ(), dt);
    return tf::Vector3(v_x, v_y, v_z);
}

// Method 2 of calculating velocity and accelereation:
double calc_vel_from_del(double dt, double x1, double x0, double vo){
    return (x1-x0)/dt;//( (2*(x1-x0)/dt) - vo);
}
tf::Vector3 v1_from_del_x(double dt, tf::Vector3 &v_o, tf::Vector3 &pos1, tf::Vector3 &pos0){
    double v1_x = calc_vel_from_del(dt, pos1.getX(), pos0.getX(), v_o.getX() );
    double v1_y = calc_vel_from_del(dt, pos1.getY(), pos0.getY(), v_o.getY() );
    double v1_z = calc_vel_from_del(dt, pos1.getZ(), pos0.getZ(), v_o.getZ() );
    return tf::Vector3(v1_x, v1_y, v1_z);
} 


double calc_a_from_v1(double dt, double v1, double vo){
    return (v1-vo)/dt;
}
tf::Vector3 a_from_v1(double dt, tf::Vector3 &v_1, tf::Vector3 &v_o) {
    double a_x = calc_a_from_v1(dt, v_1.getX(), v_o.getX() );
    double a_y = calc_a_from_v1(dt, v_1.getY(), v_o.getY() );    
    double a_z = calc_a_from_v1(dt, v_1.getZ(), v_o.getZ() );
    return tf::Vector3(a_x, a_y, a_z);    
}

// Calculates the forcing function f_target_s according to eq(8)
tf::Vector3 calc_f_target(double tau, tf::Vector3 &a_i, tf::Vector3 &v_i, tf::Vector3 &pos_i, tf::Vector3 &pos_init, 
                    tf::Vector3 &pos_goal, double K, double D, double s){
    double f_target_s_x = ((tau*a_i.getX() + D*v_i.getX())/K) - (pos_goal.getX() - pos_i.getX()) + (pos_goal.getX()-pos_init.getX())*s;
    double f_target_s_y = ((tau*a_i.getY() + D*v_i.getY())/K) - (pos_goal.getY() - pos_i.getY()) + (pos_goal.getY()-pos_init.getY())*s;    
    double f_target_s_z = ((tau*a_i.getZ() + D*v_i.getZ())/K) - (pos_goal.getZ() - pos_i.getZ()) + (pos_goal.getZ()-pos_init.getZ())*s;

    //std::cout << f_target_s_y << std::endl;
    return tf::Vector3(f_target_s_x, f_target_s_y, f_target_s_z);
}

// Calculates the forcing function f_target_s according to eq(5)
tf::Vector3 calc_f_target2(double tau, tf::Vector3 &a_i, tf::Vector3 &v_i, tf::Vector3 &pos_i, tf::Vector3 &pos_init, 
                    tf::Vector3 &pos_goal, double K, double D, double s){
    double f_target_s_x = (-K*(pos_goal.getX() - pos_i.getX()) + D*v_i.getX() + tau*a_i.getX())/(pos_goal.getX()-pos_init.getX());
    double f_target_s_y = (-K*(pos_goal.getY() - pos_i.getY()) + D*v_i.getY() + tau*a_i.getY())/(pos_goal.getY()-pos_init.getY());
    double f_target_s_z = (-K*(pos_goal.getZ() - pos_i.getZ()) + D*v_i.getZ() + tau*a_i.getZ())/(pos_goal.getZ()-pos_init.getZ());

//    std::cout << f_target_s_y << std::endl;
    return tf::Vector3(f_target_s_x, f_target_s_y, f_target_s_z);
}



// Fully debugged. Working.
// Do a linear interpolation between points and return f_target_s
tf::Vector3 f_query(double s_des, std::vector<double> &s, std::vector<tf::Vector3> &f_s){
    int i = 0;
    double s_lowerbound = 0;
    for(std::vector<double>::iterator s_i = s.begin(); s_i != s.end(); ++s_i) {
//        std::cout << *s_i << std::endl;
        // These are changed to integers so that a comparison can be made between integers
        int s_query = (int) ( s_des*10000.0 );
        int s_cur = (int) ( (*s_i)*10000.0 );                  

        // If the query is the same as the entry in the (s,f(s)) table, return f(s) from the table 
        if (s_query == s_cur){ 
            return f_s[i];
        }
        //  Note that vector s is arranged from 1 approaching 0. So long as s_query is smaller than the current s,
        //  we need to keep updating the lower bound of the linear approximation
        if (s_des < *s_i){
            i++;
            s_lowerbound = *s_i;                
            continue;
        }

        // Once we've identified that s_query is between two elements in the table, we return the linear interpolation inbetween
        if (s_des > *s_i){
            double f_s_x = ( ((f_s[i].getX() - f_s[i-1].getX()) / (s_lowerbound - *s_i))*(s_lowerbound - s_des) ) + f_s[i-1].getX() ;
            double f_s_y = ( ((f_s[i].getY() - f_s[i-1].getY()) / (s_lowerbound - *s_i))*(s_lowerbound - s_des) ) + f_s[i-1].getY() ;            
            double f_s_z = ( ((f_s[i].getZ() - f_s[i-1].getZ()) / (s_lowerbound - *s_i))*(s_lowerbound - s_des) ) + f_s[i-1].getZ() ;
           // std::cout << f_s_y << std::endl;

            return tf::Vector3(f_s_x, f_s_y, f_s_z);
        }

        //std::cout << "Loop Check" << std::endl;            
    }

    // Reached the end of f(s). Return the last element of f(s)
    //    std::cout << i << std::endl;
    //    std::cout << f_s[i-1].getX() << std::endl; // Since we did a i++ before ending the loop

//    return tf::Vector3(0,0,0);
    return tf::Vector3(f_s[i-1].getX(), f_s[i-1].getY(), f_s[i-1].getZ());

}

// Calculate accelerations from eq(6)
double calculate_acel(double tau, double K, double D, double goal_pos, double cur_pos, double cur_vel, double start_pos, double s_des, double fs){
//    double acel = (1/tau) * ( K*(goal_pos- cur_pos) - D*cur_vel - K*(goal_pos - start_pos)*s_des + K*fs);
    double acel = (1/tau) * ( K*((goal_pos-cur_pos)-(goal_pos - start_pos)*s_des + fs) - D*cur_vel );
    return acel;
}


// Calculate accelerations, but from eq (1)
double calculate_acel2(double tau, double K, double D, double goal_pos, 
                        double cur_pos, double cur_vel, double start_pos, double s_des, double fs){
    double acel = (1/tau) * ( K*(goal_pos- cur_pos) - D*cur_vel - 
                                    (goal_pos - start_pos)*fs);
    return acel;
}

// Calculate the waypoints via integration of the dynamic system
std::vector<tf::Vector3> generate_waypoints(double K, double D, double tau, double alpha,  tf::Vector3 &start_pos, 
                                                                                           tf::Vector3 &goal_pos,
                                                                                           std::vector<double> &s, 
                                                                                           std::vector<tf::Vector3> &f_s,
                                                                                           int n_samples,
                                                                                           std::vector<double> demo_t) {
    double dt = 0.001;
    double t = 0;
    int iters = (int) (tau/dt);
    double s_cur = exp(-alpha/tau*t);

    std::vector<tf::Vector3> h;
    tf::Vector3 vel(0,0,0);       
    tf::Vector3 pos(start_pos.getX(), start_pos.getY(), start_pos.getZ());       

    for (std::vector<int>::size_type i = 0; i < iters; ++i){        
//   for (std::vector<int>::size_type i = 1; i < n_samples; ++i){
//        double dt = demo_t[i] - demo_t[i-1];
        double s_des = exp(-alpha/tau * t);
//        double acel_x = (1/tau) * ( K*(goal_pos.getX() - pos.getX()) - D*vel.getX() - 
                                    // K*(goal_pos.getX() - start_pos.getX())*s_des +  
                                    // K*f_query(s_des, s, f_s).getX() );

        double acel_x = calculate_acel(tau, K, D, goal_pos.getX(), pos.getX(), vel.getX(), start_pos.getX(), s_des, f_query(s_des, s, f_s).getX());
        double acel_y = calculate_acel(tau, K, D, goal_pos.getY(), pos.getY(), vel.getY(), start_pos.getY(), s_des, f_query(s_des, s, f_s).getY());
        double acel_z = calculate_acel(tau, K, D, goal_pos.getZ(), pos.getZ(), vel.getZ(), start_pos.getZ(), s_des, f_query(s_des, s, f_s).getZ());

        double vel_x = (acel_x*dt + vel.getX());
        double vel_y = (acel_y*dt + vel.getY());
        double vel_z = (acel_z*dt + vel.getZ());

        double pos_x = (1/tau)*vel_x*dt + pos.getX();
        double pos_y = (1/tau)*vel_y*dt + pos.getY();
        double pos_z = (1/tau)*vel_z*dt + pos.getZ();

//        std::cout << pos_x << std::endl;
        std::cout << pos_y << std::endl;        
//        std::cout << t << std::endl;
//        std::cout << f_query(s_des, s, f_s).getY() << std::endl;        

        pos = tf::Vector3(pos_x, pos_y, pos_z);
        vel = tf::Vector3(vel_x, vel_y, vel_z);        

        h.push_back(pos);

        t += dt;     
   }
    return h;
}

int main(int argc, char **argv){
    ros::init (argc, argv, "dmp_f_target_calc");
    ros::NodeHandle n;
    ros::Publisher rvizMarkerPub; 
    rvizMarkerPub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1000);

//    ROS_INFO("Opening Bag");
    rosbag::Bag bag;    
    bag.open("reach.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("visualization_marker")); //Specify topic to read
    rosbag::View view(bag, rosbag::TopicQuery(topics));


    // =======================================================================================================================================
    // Parse rosbag
    std::vector<double> demo_t;
    std::vector<tf::Vector3> demo_pos;
    std::vector<tf::Vector3> demo_vel;
    std::vector<tf::Vector3> demo_acel;

    int start_time_s = 0;
    double start_nsec = 0;
    int n_samples = 0;

    // Go to the trajectory bag for the first time.
    foreach(rosbag::MessageInstance const m, view){       
        visualization_msgs::Marker::ConstPtr p = m.instantiate<visualization_msgs::Marker>();
        // Store marker x,y,z cartesian pose.        
        tf::Vector3 marker_cartesian_pose( p->pose.position.x,  p->pose.position.y,  p->pose.position.z);
        demo_pos.push_back(marker_cartesian_pose);

        // Store time variable;
        if (n_samples == 0) {
            start_time_s = p->header.stamp.sec; //Obtain first time stamp
            start_nsec = (double) (p->header.stamp.nsec)/pow(10,9);    // Bring nanosec to seconds.     
        }
        double sec = (double) (p->header.stamp.sec - start_time_s); // Subtract time stamp to get sensible seconds in int then convert to double
        double nsec = (double) (p->header.stamp.nsec)/pow(10,9);    // Bring nanosec to seconds.     
        demo_t.push_back(sec + nsec - start_nsec); // Store the true time stamp

        // Count total number of markers in the rosbag
        n_samples++; 
    }    
    //std::cout << n_samples << std::endl;
    // =======================================================================================================================================

    // =======================================================================================================================================
    // Calculate velocity and acceleration
        // Note some math functions:
        //        std::cout << pow(10,2) << std::endl;
        //        std::cout << log(1) << std::endl; 
        //        std::cout << exp(1) << std::endl;
    // Initialize Velocity(0) to 0    
    tf::Vector3 v_o(0,0,0);
    demo_vel.push_back(v_o);
    

    // Both differentiation method works
    //Method 1:
    // for (std::vector<int>::size_type i = 0; i < n_samples-1; ++i){
    //     //std::cout << demo_pos[0].getX() << std::endl;
    //     double dt = demo_t[i+1] - demo_t[i];
    //     // Calculate constant acceleration(i) to get xo to x1 and add it to acceleration vector.
    //     demo_acel.push_back( constant_a(demo_pos[i+1], demo_pos[i], demo_vel[i], dt) );
    //     // Calculate velocity(i+1) and add it to velocity vector.
    //     demo_vel.push_back( vel_from_a(demo_acel[i], demo_vel[i], dt) );
    // }

    // Method 2:
    for (std::vector<int>::size_type i = 1; i < n_samples; ++i){
        //std::cout << demo_pos[0].getX() << std::endl;
        double dt = demo_t[i] - demo_t[i-1];
        tf::Vector3 v_1 = v1_from_del_x(dt, v_o, demo_pos[i], demo_pos[i-1]);
        demo_acel.push_back( a_from_v1(dt, v_1, v_o) );
        v_o = tf::Vector3(v_1.getX(), v_1.getY(), v_1.getZ());
        demo_vel.push_back(v_1);
    }


    // =======================================================================================================================================

    // =======================================================================================================================================
    // Calculate DMP Part 1: get f_target(s)
    double K = 500.0;
    double D = 2*sqrt(K);
    double tau = 10; // in seconds


    std::vector<double> phase_s;
    std::vector<tf::Vector3> f_target_s;
    tf::Vector3 demo_start = demo_pos[0]; //start position is set to be the first demo x,y,z position    
    tf::Vector3 demo_goal = demo_pos[n_samples-1]; //goal is set to be the last demo x,y,z position
    double tau_demo = demo_t[n_samples-1] - demo_t[0]; // duration of demo
    double alpha = -log(0.01); // selected so that s(t) = exp(-alpha/tau_demo)*t converges to 99% when t = tau_demo    

//    std::cout << tau_demo << std::endl;
//    std::cout << alpha << std::endl;
    for (std::vector<int>::size_type i = 0; i < n_samples-1; ++i){
        double s = exp(-alpha/tau_demo * demo_t[i]); //Grab current time and find s
        phase_s.push_back(s); // store phase variable s
//        std::cout << s << std::endl;
        // calculate f_target(s) 
        f_target_s.push_back(  calc_f_target(tau_demo, demo_acel[i], demo_vel[i], demo_pos[i], demo_start, demo_goal, K, D, s) ); 

    }
    // =======================================================================================================================================
    // Calculate DMP part 2: Generate xyz waypoints
    double tau_des = tau_demo; // Set duration of copying the trajectory

    tf::Vector3 r_gripper_start_pos(0,0,0); //Modify this to pr2's starting arm position
    tf::Vector3 r_gripper_goal_pos(0.1223,-0.06,0); //Modify this to pr2's starting arm position
//    tf::Vector3 r_gripper_goal_pos(0.2123,-0.06,0); //Modify this to pr2's starting arm position
//    tf::Vector3 r_gripper_goal_pos(0.2223,-0.07,0); //Modify this to pr2's starting arm position

    std::vector<tf::Vector3> xyz_waypoints = generate_waypoints(K, D, tau_des, alpha, r_gripper_start_pos, 
                                                                                      r_gripper_goal_pos, 
                                                                                      phase_s, 
                                                                                      f_target_s,
                                                                                      n_samples,
                                                                                      demo_t); 

    // std::vector<tf::Vector3> xyz_waypoints = generate_waypoints(K, D, tau_des, alpha, demo_pos[0], 
    //                                                                                   demo_pos[n_samples-1], 
    //                                                                                   phase_s, 
    //                                                                                   f_target_s,
    //                                                                                   n_samples,
    //                                                                                   demo_t);     

// std::cout << "The y positions before were:" << std::endl;
//   for (std::vector<int>::size_type i = 0; i < n_samples; ++i){
//       std::cout<< demo_pos[i].getY() << std::endl;
//   }    

/*
   // Define Reference quaternion to be the x-axis.
    tf::Quaternion main_axis(1,0,0, 1);
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

        pub_recorded_marker(rvizMarkerPub, p, marker_index, total_markers, transform_to_main_axis, rotate_to_main_axis);
        marker_index++;
        //sleep(1.0);
        std::cout << total_markers << std::endl;
    }
*/
 //   ROS_INFO("Closing bag");
    bag.close();


// ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
// gazebo_msgs::GetModelState getmodelstate;
// getmodelstate.request.model_name = "table_1";
// gms_c.call(getmodelstate);


// //ros::NodeHandle n;
// geometry_msgs::Pose start_pose;
// start_pose.position.x = getmodelstate.response.pose.position.x;
// start_pose.position.y = getmodelstate.response.pose.position.y;
// start_pose.position.z = can_location.getZ()-TABLE_CAN_Z_DIFF; //getmodelstate.response.pose.position.z;
// start_pose.orientation.x = 0.0;
// start_pose.orientation.y = 0.0;
// start_pose.orientation.z = 0.0;
// start_pose.orientation.w = 0.0;

// geometry_msgs::Twist start_twist;
// start_twist.linear.x = 0.0;
// start_twist.linear.y = 0.0;
// start_twist.linear.z = 0.0;
// start_twist.angular.x = 0.0;
// start_twist.angular.y = 0.0;
// start_twist.angular.z = 0.0;

// gazebo_msgs::ModelState modelstate;
// modelstate.model_name = (std::string) "table_1";
// modelstate.reference_frame = (std::string) "world";
// modelstate.pose = start_pose;
// modelstate.twist = start_twist;

// ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
// gazebo_msgs::SetModelState setmodelstate;
// setmodelstate.request.model_state = modelstate;
// client.call(setmodelstate);



// std::cout << get_can_pos(n).getX() << std::endl;
// std::cout << get_can_pos(n).getY() << std::endl;
// std::cout << get_can_pos(n).getZ() << std::endl;

// ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
// gazebo_msgs::GetModelState getmodelstate;

//getmodelstate.request.model_name = "coke_can";
// gms_c.call(getmodelstate);

// std::cout << getmodelstate.response.pose.position.x << std::endl;
// std::cout << getmodelstate.response.pose.position.y << std::endl;
// std::cout << getmodelstate.response.pose.position.z << std::endl;


// ros::ServiceClient gls_c = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
// gazebo_msgs::GetLinkState getlinkstate;

//  getlinkstate.request.link_name = "pr2::r_gripper_l_finger_tip_link";
//  gls_c.call(getlinkstate);

//  std::cout << getlinkstate.response.link_state.pose.position.x << std::endl;
//  std::cout << getlinkstate.response.link_state.pose.position.y << std::endl;
//  std::cout << getlinkstate.response.link_state.pose.position.z << std::endl;



// tf::Quaternion q;
// q.setRPY(1.57,0,0);
// modelstate.model_name = (std::string) "coke_can";
// modelstate.reference_frame = (std::string) "world";
// start_pose.position.x = getmodelstate.response.pose.position.x;
// start_pose.position.y = getmodelstate.response.pose.position.y;
// start_pose.position.z = getmodelstate.response.pose.position.z;
// start_pose.orientation.x = q.getAxis().getX();
// start_pose.orientation.y = q.getAxis().getY();
// start_pose.orientation.z = q.getAxis().getZ();
// start_pose.orientation.w = q.getW();
// modelstate.pose = start_pose;
// setmodelstate.request.model_state = modelstate;
// //modelstate.twist = start_twist;
// client.call(setmodelstate);

//pr2::r_gripper_l_finger_tip_link

// (0,0,0.051) is the height of the base_link from the ground 


}