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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>


#define foreach BOOST_FOREACH

#define MARKER_ID_TO_TRACK 4
#define MARKER_ID_TO_REORIENT 6
#define DEMO_FILE_NAME "/home/stevenjj/catkin_ws/src/hw3/demo_bags/lift_box_4.bag"
#define K_CONSTANT 500.0
#define TABLE_CAN_Z_DIFF 1.9328

struct DMP_param{
    double K;
    double D;
    double tau_demo;
    double alpha;
    int n_samples;

    std::vector<tf::Vector3> demo_t;

    std::vector<tf::Vector3> demo_pos;
    std::vector<tf::Vector3> demo_vel;
    std::vector<tf::Vector3> demo_acel;    

    std::vector<double> s;
    std::vector<tf::Vector3> f_s;

    void replace_DMP_with(DMP_param new_dmp){
        this->K = new_dmp.K;
        this->D = new_dmp.D;
        this->tau_demo = new_dmp.tau_demo;
        this->alpha = new_dmp.alpha;
        this->n_samples = new_dmp.n_samples;

        this->demo_t = new_dmp.demo_t;
        
        this->demo_pos = new_dmp.demo_pos;                                
        this->demo_vel = new_dmp.demo_vel;
        this->demo_acel = new_dmp.demo_acel;

        this->s = new_dmp.s;
        this->f_s = new_dmp.f_s;                        
    }

}; // No global objects initialized

struct DMP_plan_traj{
    std::vector<double> t; // Time stamp
    std::vector<tf::Vector3> cart_pos_plan;
};

struct DEMO_traj{
    std::vector<double> time;
    std::vector<tf::Vector3> pos;
    std::vector<tf::Vector3> vel;
    std::vector<tf::Vector3> acel;        
    int n_samples; // Number of samples

    tf::Vector3 start_pos;
    tf::Vector3 goal_pos;    

};

struct Waypoints_traj{
    std::vector<double> time;
    std::vector<tf::Vector3> pos;

};

tf::Vector3 pr2_traj_to_dmp_frame(tf::Vector3 vector_pos){
    return tf::Vector3(vector_pos.getY(), -vector_pos.getZ(), -vector_pos.getX() );
}

tf::Vector3 dmp_to_pr2_frame(tf::Vector3 vector_pos){
    return tf::Vector3(-vector_pos.getZ(), vector_pos.getX(), -vector_pos.getY() );
}


double getPhase(double alpha, double tau, double t);

void create_pr2_waypoints(int index, const Waypoints_traj &des_waypoints, std::vector<geometry_msgs::Pose> &cartesian_waypoints, 
                          const tf::Transform &translate_to_main, const tf::Transform &rotate_to_main){

   tf::Vector3 xyz_position (des_waypoints.pos[index].getX(),
                                 des_waypoints.pos[index].getY(),
                                 des_waypoints.pos[index].getZ());
    tf::Quaternion xyz_orientation (1, 0, 0, 0);

    //marker_position = marker_position - offset_vector; //transform_to_main * 
    xyz_position = translate_to_main * xyz_position; 
    xyz_orientation = rotate_to_main * xyz_orientation;

// Push marker 6DoF Pose to the waypoints vector.
    geometry_msgs::Pose target_pose;
    target_pose.position.x = xyz_position.getX();
    target_pose.position.y = xyz_position.getY();
    target_pose.position.z = xyz_position.getZ();

    target_pose.orientation.x = xyz_orientation.getAxis().getX();//rosbag_marker->pose.orientation.x;//0.0;
    target_pose.orientation.y = xyz_orientation.getAxis().getY();//rosbag_marker->pose.orientation.y;//0.0;
    target_pose.orientation.z = xyz_orientation.getAxis().getZ();//rosbag_marker->pose.orientation.z;//0.0;
    target_pose.orientation.w = xyz_orientation.getW();//rosbag_marker->pose.orientation.w; //1.0;

    cartesian_waypoints.push_back(target_pose);

}

void pub_dmp_markers(ros::Publisher &marker_pub, int index, int total_markers, Waypoints_traj &des_waypoints,
                    tf::Transform &translate_to_main, tf::Transform &rotate_to_main){

    tf::Vector3 marker_position (des_waypoints.pos[index].getX(),
                                 des_waypoints.pos[index].getY(),
                                 des_waypoints.pos[index].getZ());
    tf::Quaternion marker_orientation (1, 0, 0, 0);

    //marker_position = marker_position - offset_vector; //transform_to_main * 
    marker_position = translate_to_main * marker_position; 
    marker_orientation = rotate_to_main * marker_orientation;

     visualization_msgs::Marker marker;
//    uint32_t shape = visualization_msgs::Marker::CUBE;
//    uint32_t shape = visualization_msgs::Marker::SPHERE;
    uint32_t shape = visualization_msgs::Marker::ARROW;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //marker.header.frame_id = "/my_frame";
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = index;//rosbag_marker->id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape; //rosbag_marker->type; 

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = marker_position.getX();//rosbag_marker->pose.position.x;//0;
    marker.pose.position.y = marker_position.getY();//rosbag_marker->pose.position.y;//0;
    marker.pose.position.z = marker_position.getZ();//rosbag_marker->pose.position.z;//0;


    // marker.pose.position.x = -marker_position.getZ();//rosbag_marker->pose.position.x;//0;
    // marker.pose.position.y = marker_position.getX();//rosbag_marker->pose.position.y;//0;
    // marker.pose.position.z = -marker_position.getY();//rosbag_marker->pose.position.z;//0;

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
    //std::cout << index << std::endl;
    //sleep(1.0);
}



double getPhase(double alpha, double tau, double t){
    return exp(-alpha/tau*t); //Grab current time and find s    
}


// METHOD 1 of velocity and acceleration
// Calculates constant acceleration between two points x0, x1 and factors in velocity between them.
double calc_acel(const double x1, const double xo, const double vo, const double dt){
    return ( (x1 - xo - (vo*dt))/pow(dt,2) );
}

// Returns acceleration for x,y,z
tf::Vector3 constant_a(const tf::Vector3 &pos_next_i, const tf::Vector3 &pos_i, const tf::Vector3 &vel_i, const double dt){
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
tf::Vector3 calc_f_target(const double tau, const tf::Vector3 &a_i, const tf::Vector3 &v_i, const tf::Vector3 &pos_i, const tf::Vector3 &pos_init, 
                    const tf::Vector3 &pos_goal, const double K, const double D, const double s){
    double f_target_s_x = ((tau*a_i.getX() + D*v_i.getX())/K) - (pos_goal.getX() - pos_i.getX()) + (pos_goal.getX()-pos_init.getX())*s;
    double f_target_s_y = ((tau*a_i.getY() + D*v_i.getY())/K) - (pos_goal.getY() - pos_i.getY()) + (pos_goal.getY()-pos_init.getY())*s;    
    double f_target_s_z = ((tau*a_i.getZ() + D*v_i.getZ())/K) - (pos_goal.getZ() - pos_i.getZ()) + (pos_goal.getZ()-pos_init.getZ())*s;

    //std::cout << f_target_s_y << std::endl;
    return tf::Vector3(f_target_s_x, f_target_s_y, f_target_s_z);
}

// Calculates the forcing function f_target_s according to eq(5)
tf::Vector3 calc_f_target2(const double tau, const tf::Vector3 &a_i, const tf::Vector3 &v_i, const tf::Vector3 &pos_i, const tf::Vector3 &pos_init, 
                    const tf::Vector3 &pos_goal, const double K, const double D, const double s){
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


// Calculate the waypoints via integration of the dynamic system. This is the test version, so it should look similar to the original demo
std::vector<tf::Vector3> test_dmp(const double K, const double D, const double tau, const double alpha,  tf::Vector3 &start_pos, 
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

    for (std::vector<int>::size_type i = 1; i < n_samples; ++i){
       double dt = demo_t[i] - demo_t[i-1];
       double s_des = getPhase(alpha, tau, t);//exp(-alpha/tau * t);

        double acel_x = calculate_acel(tau, K, D, goal_pos.getX(), pos.getX(), vel.getX(), start_pos.getX(), s_des, f_query(s_des, s, f_s).getX());
        double acel_y = calculate_acel(tau, K, D, goal_pos.getY(), pos.getY(), vel.getY(), start_pos.getY(), s_des, f_query(s_des, s, f_s).getY());
        double acel_z = calculate_acel(tau, K, D, goal_pos.getZ(), pos.getZ(), vel.getZ(), start_pos.getZ(), s_des, f_query(s_des, s, f_s).getZ());

        double vel_x = (acel_x*dt + vel.getX());
        double vel_y = (acel_y*dt + vel.getY());
        double vel_z = (acel_z*dt + vel.getZ());

        double pos_x = (1/tau)*vel_x*dt + pos.getX();
        double pos_y = (1/tau)*vel_y*dt + pos.getY();
        double pos_z = (1/tau)*vel_z*dt + pos.getZ();

        std::cout << pos_x ;
        std::cout << " " ;
        std::cout << -pos_x ;
        std::cout << " " ;        
        std::cout << pos_y ;        
        std::cout << " " ;        
        std::cout << pos_z << std::endl;        

        pos = tf::Vector3(pos_x, pos_y, pos_z);
        vel = tf::Vector3(vel_x, vel_y, vel_z);        

        h.push_back(pos);

        t += dt;     
   }
    return h;
}


void print_demo_traj(DEMO_traj &demo_data){
  for (std::vector<int>::size_type i = 0; i < demo_data.n_samples; ++i){
      std::cout<< demo_data.pos[i].getX();
      std::cout<< " "; 
      std::cout<< demo_data.pos[i].getY() << std::endl;
  }    
}

void obtain_pos_data_from_bag(std::string file_name, int marker_id_to_learn, DEMO_traj &demo_data){
//    ROS_INFO("Opening Bag");
    rosbag::Bag bag;    
    bag.open(file_name, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("visualization_marker")); //Specify topic to read
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // =======================================================================================================================================
    // Parse rosbag
    int start_time_s = 0;
    double start_nsec = 0;
    int n_samples = 0;

    int marker_id = marker_id_to_learn;
    // Go to the trajectory bag for the first time.
    foreach(rosbag::MessageInstance const m, view){       
        visualization_msgs::Marker::ConstPtr p = m.instantiate<visualization_msgs::Marker>();
        if (p->id == marker_id){
            // Store marker x,y,z cartesian pose.        
            tf::Vector3 marker_cartesian_pose( p->pose.position.x,  p->pose.position.y,  p->pose.position.z);
//            tf::Vector3 marker_cartesian_pose( -(p->pose.position.z),  p->pose.position.z, - (p->pose.position.x) ;            
            demo_data.pos.push_back(marker_cartesian_pose);

            // Store time variable;
            if (n_samples == 0) {
                start_time_s = p->header.stamp.sec; //Obtain first time stamp
                start_nsec = (double) (p->header.stamp.nsec)/pow(10,9);    // Bring nanosec to seconds.     
            }
            double sec = (double) (p->header.stamp.sec - start_time_s); // Subtract time stamp to get sensible seconds in int then convert to double
            double nsec = (double) (p->header.stamp.nsec)/pow(10,9);    // Bring nanosec to seconds.     
            demo_data.time.push_back(sec + nsec - start_nsec); // Store the true time stamp

            // Count total number of markers in the rosbag
            n_samples++; 
        }
    }
    demo_data.n_samples = n_samples;

    demo_data.start_pos = demo_data.pos[0];
    demo_data.goal_pos = demo_data.pos[n_samples-1];   

    bag.close(); 
}

tf::Vector3 obtain_demo_object_start_pos(std::string file_name, int object_marker_id, DEMO_traj &demo_data){
    rosbag::Bag bag;    
    bag.open(file_name, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("visualization_marker")); //Specify topic to read
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    tf::Vector3 object_demo_cartesian_pose;
    int marker_id = object_marker_id;
    int i = 0;
    // Go to the trajectory bag for the first time.
    foreach(rosbag::MessageInstance const m, view){       
        visualization_msgs::Marker::ConstPtr p = m.instantiate<visualization_msgs::Marker>();
        if ((p->id == marker_id) && (i == 0)){
            // Store object's marker x,y,z cartesian pose.        
            object_demo_cartesian_pose.setX(p->pose.position.x);
            object_demo_cartesian_pose.setY(p->pose.position.y);
            object_demo_cartesian_pose.setZ(p->pose.position.y);                        
            i++;
        }
    }
    bag.close(); 
    // Find the difference and return offset in the pr2 frame.
    return dmp_to_pr2_frame(demo_data.pos[(demo_data.n_samples)-1] - object_demo_cartesian_pose);

}

void calculate_vel_acel_data(DEMO_traj &demo_data){
    tf::Vector3 v_o(0,0,0);
    demo_data.vel.push_back(v_o);
    
//    Both differentiation method works
//    Method 1: Constant acceleration
    // for (std::vector<int>::size_type i = 0; i < demo_data.n_samples-1; ++i){
    //     //std::cout << demo_pos[0].getX() << std::endl;
    //     double dt = demo_data.time[i+1] - demo_data.time[i];
    //     // Calculate constant acceleration(i) to get xo to x1 and add it to acceleration vector.
    //     demo_data.acel.push_back( constant_a(demo_data.pos[i+1], demo_data.pos[i], demo_data.vel[i], dt) );
    //     // Calculate velocity(i+1) and add it to velocity vector.
    //     demo_data.vel.push_back( vel_from_a(demo_data.acel[i], demo_data.vel[i], dt) );
    // }

    // Method 2: naive dx/dt and dv/dt
    for (std::vector<int>::size_type i = 1; i < demo_data.n_samples; ++i){
        //std::cout << demo_pos[0].getX() << std::endl;
        double dt = demo_data.time[i] - demo_data.time[i-1];
        tf::Vector3 v_1 = v1_from_del_x(dt, v_o, demo_data.pos[i], demo_data.pos[i-1]);
        demo_data.acel.push_back( a_from_v1(dt, v_1, v_o) );
        v_o = tf::Vector3(v_1.getX(), v_1.getY(), v_1.getZ());
        demo_data.vel.push_back(v_1);
    }

}


void dmp_learning(const double K, const double D, double alpha, const DEMO_traj &traj_demo_to_learn,  DMP_param &dmp_store){
    int n_samples = traj_demo_to_learn.n_samples; //
    double tau_demo = traj_demo_to_learn.time[n_samples-1]; //Grab the last element of the time stamped demo trajectory.

    dmp_store.K = K;
    dmp_store.D = D;
    dmp_store.alpha = alpha;
    dmp_store.tau_demo = tau_demo; 
    dmp_store.n_samples = n_samples;

    for (std::vector<int>::size_type i = 0; i < (n_samples - 1); ++i){
        double s = getPhase(alpha, tau_demo, traj_demo_to_learn.time[i]); //Grab current time and find s                
        dmp_store.s.push_back(s); // store phase variable s
        // calculate f_target(s) 
        dmp_store.f_s.push_back(  calc_f_target(tau_demo, traj_demo_to_learn.acel[i], 
                                                          traj_demo_to_learn.vel[i], 
                                                          traj_demo_to_learn.pos[i], 
                                                          traj_demo_to_learn.start_pos,
                                                          traj_demo_to_learn.goal_pos,
                                                          K, D, s) ) ; 
     }

}

void dmp_planning(double tau, double dt_des, const tf::Vector3 &start_pos, const tf::Vector3 &goal_pos, 
                                                 const DMP_param &dmp_to_use, Waypoints_traj &waypoints){
        double K = dmp_to_use.K;
        double D = dmp_to_use.D;        
        double alpha = dmp_to_use.alpha;

        std::vector<double> s = dmp_to_use.s;
        std::vector<tf::Vector3> f_s = dmp_to_use.f_s;

        double dt = dt_des;
        double t = 0;
        int iters = (int) (tau/dt);
        double s_cur = getPhase(alpha, tau, t);

        tf::Vector3 vel(0,0,0);       
        tf::Vector3 pos(start_pos.getX(), start_pos.getY(), start_pos.getZ());       

     for (std::vector<int>::size_type i = 0; i < iters; ++i){        
        waypoints.time.push_back(t);
        double s_des = getPhase(alpha, tau, t);//exp(-alpha/tau * t);

        double acel_x = calculate_acel(tau, K, D, goal_pos.getX(), pos.getX(), vel.getX(), start_pos.getX(), s_des, f_query(s_des, s, f_s).getX());
        double acel_y = calculate_acel(tau, K, D, goal_pos.getY(), pos.getY(), vel.getY(), start_pos.getY(), s_des, f_query(s_des, s, f_s).getY());
        double acel_z = calculate_acel(tau, K, D, goal_pos.getZ(), pos.getZ(), vel.getZ(), start_pos.getZ(), s_des, f_query(s_des, s, f_s).getZ());

        double vel_x = (acel_x*dt + vel.getX());
        double vel_y = (acel_y*dt + vel.getY());
        double vel_z = (acel_z*dt + vel.getZ());

        double pos_x = (1/tau)*vel_x*dt + pos.getX();
        double pos_y = (1/tau)*vel_y*dt + pos.getY();
        double pos_z = (1/tau)*vel_z*dt + pos.getZ();

        // std::cout << pos_x ;
        // std::cout << " " ;
        // std::cout << pos_y << std::endl;        

        pos = tf::Vector3(pos_x, pos_y, pos_z);
        vel = tf::Vector3(vel_x, vel_y, vel_z);        

        waypoints.pos.push_back(pos);

         t += dt;     
    }
 }

 void print_waypoints(Waypoints_traj &waypoints) {
    int n = 0;
    for(std::vector<double>::iterator t_i = waypoints.time.begin(); t_i != waypoints.time.end(); ++t_i) {
        n++;    // Count number of elements in waypoints    
    }    
    
    std::cout << "time x -x y z" << std::endl;
    
    for (std::vector<int>::size_type i = 0; i < n; ++i){
        std::cout << waypoints.time[i];
        std::cout << " " ;
        std::cout << waypoints.pos[i].getX();
        std::cout << " " ;
        std::cout << -waypoints.pos[i].getX();
        std::cout << " " ;        
        std::cout << waypoints.pos[i].getY();        
        std::cout << " " ;
        std::cout << waypoints.pos[i].getZ() << std::endl;                  
    }    

    // for(std::vector<tf::Vector3>::iterator pos_i = waypoints.pos.begin(); pos_i != waypoints.pos.end(); ++pos_i) {
    //     std::cout << (*pos_i).getX();
    //     std::cout << " " ;
    //     std::cout << (*pos_i).getY();        
    //     std::cout << " " ;
    //     std::cout << (*pos_i).getZ() << std::endl;                
    // }
 }

void learn_and_plan_dmp_trajectory(DMP_param &reaching_dmp, DEMO_traj &reaching_demo_traj, Waypoints_traj &des_waypoints, const tf::Vector3 &goal_offset_vector){
    // Learn and plan DMP trajectory -------------------------------------------------------------------------------------
    std::string demo_file = DEMO_FILE_NAME;
    int marker_id_to_learn = MARKER_ID_TO_TRACK;
    obtain_pos_data_from_bag(demo_file, marker_id_to_learn, reaching_demo_traj);  // Grab data
    calculate_vel_acel_data(reaching_demo_traj); // Calculate velocities and accelerations
    //    print_demo_traj(reaching_demo_traj); //print out recorded trajectories

    double K = K_CONSTANT;
    double D = 2*sqrt(K);
    double alpha = -log(0.01); // selected so that s(t) = exp(-alpha/tau_demo)*t converges to 99% when t = tau_demo        
    dmp_learning(K, D, alpha, reaching_demo_traj, reaching_dmp); // Learn the dmp of reaching demo

    // Prep the dmp planning phase
    tf::Vector3 r_gripper_start_pos(0,0,0); //Always 0 since real starting position is set by moveit.
    int traj_samples = reaching_demo_traj.n_samples;
    double goal_x = reaching_demo_traj.pos[traj_samples-1].getX() - reaching_demo_traj.pos[0].getX()  ;
    double goal_y = reaching_demo_traj.pos[traj_samples-1].getY() - reaching_demo_traj.pos[0].getY()  ;
    double goal_z = reaching_demo_traj.pos[traj_samples-1].getZ() - reaching_demo_traj.pos[0].getZ()  ;

    tf::Vector3 task_goal_pos(goal_x, goal_y, goal_z);    
    tf::Vector3 r_gripper_goal_pos = task_goal_pos + goal_offset_vector;

    // tf::Vector3 r_gripper_goal_pos(-0.164,-0.055,-0.232);
    //tf::Vector3 r_gripper_goal_pos(goal_x, goal_y, goal_z);    
    double dt_des = 0.01;
    double tau_des = reaching_dmp.tau_demo; // Set duration of copying the trajectory    

    dmp_planning(tau_des, dt_des, r_gripper_start_pos, r_gripper_goal_pos, reaching_dmp, des_waypoints);
    //print_waypoints(des_waypoints); // Print DMP learned traj

    // Test DMP
    // std::vector<tf::Vector3> xyz_waypoints = test_dmp(K, D, reaching_dmp.tau_demo, alpha, reaching_demo_traj.start_pos, 
    //                                                                                   reaching_demo_traj.goal_pos, 
    //                                                                                   reaching_dmp.s, 
    //                                                                                   reaching_dmp.f_s,
    //                                                                                   reaching_dmp.n_samples,
    //                                                                                   reaching_demo_traj.time); 
    // End of planning DMP trajectory -------------------------------------------------------------------------------------
}

void learn_dmp(DMP_param &reaching_dmp, DEMO_traj &reaching_demo_traj){
    // Learn and plan DMP trajectory -------------------------------------------------------------------------------------
    std::string demo_file = DEMO_FILE_NAME;
    int marker_id_to_learn = MARKER_ID_TO_TRACK;
    obtain_pos_data_from_bag(demo_file, marker_id_to_learn, reaching_demo_traj);  // Grab data
    calculate_vel_acel_data(reaching_demo_traj); // Calculate velocities and accelerations
    //    print_demo_traj(reaching_demo_traj); //print out recorded trajectories

    double K = K_CONSTANT;
    double D = 2*sqrt(K);
    double alpha = -log(0.01); // selected so that s(t) = exp(-alpha/tau_demo)*t converges to 99% when t = tau_demo        
    dmp_learning(K, D, alpha, reaching_demo_traj, reaching_dmp); // Learn the dmp of reaching demo and store parameters by mutating reaching_dmp
}


// learns a dmp, and plans a dmp for a goal 
void  plan_dmp_trajectory(DMP_param &dmp_to_use, Waypoints_traj &des_waypoints, const tf::Vector3 &goal_vector){
    // Prep the dmp planning phase
    tf::Vector3 r_gripper_start_pos(0,0,0); //Always 0 since real starting position is set by moveit.
    tf::Vector3 r_gripper_goal_pos = goal_vector;    

    double dt_des = 0.5;
    double tau_des = dmp_to_use.tau_demo; // Set duration of copying the trajectory    
    dmp_planning(tau_des, dt_des, r_gripper_start_pos, r_gripper_goal_pos, dmp_to_use, des_waypoints); //mutates des_waypoints
    //print_waypoints(des_waypoints); // Print DMP learned traj    
}





void convert_waypoints_to_pr2_axes(Waypoints_traj &waypoints, Waypoints_traj &pr2_waypoints) {
    // All we're doing is making x negative. But I want to transform the waypoints after the dmp is learned to keep
    // pr2 and dmp implementation separated.
    int n = 0;
    for(std::vector<double>::iterator t_i = waypoints.time.begin(); t_i != waypoints.time.end(); ++t_i) {
        n++;    // Count number of elements in waypoints    
    }    
        
    for (std::vector<int>::size_type i = 0; i < n; ++i){
        // transform back to pr2 coordinates.       
        tf::Vector3 pr2_points = dmp_to_pr2_frame(waypoints.pos[i]);
        pr2_waypoints.time.push_back(waypoints.time[i]);
        pr2_waypoints.pos.push_back(pr2_points);        
    }    
}

tf::Vector3 get_r_gripper_pos(ros::NodeHandle &n){
    ros::ServiceClient gls_c = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    gazebo_msgs::GetLinkState getlinkstate;

    getlinkstate.request.link_name = "pr2::r_gripper_l_finger_tip_link";
    gls_c.call(getlinkstate);

    double x = getlinkstate.response.link_state.pose.position.x;
    double y = getlinkstate.response.link_state.pose.position.y;
    double z = getlinkstate.response.link_state.pose.position.z;

    std::cout << x << std::endl;
    std::cout << y << std::endl;
    std::cout << z << std::endl;

    return tf::Vector3(x,y,z); //returns gripper position with the base link as the origin.
}

tf::Vector3 get_object_pos(ros::NodeHandle &n){
    ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getmodelstate;

    getmodelstate.request.model_name = "block";
    gms_c.call(getmodelstate);

    double x = getmodelstate.response.pose.position.x;
    double y = getmodelstate.response.pose.position.y;
    double z = getmodelstate.response.pose.position.z;

    // std::cout << x << std::endl;
    // std::cout << y << std::endl;
    // std::cout << z << std::endl;    

    return tf::Vector3(x,y,z); 
}

void set_object_location(ros::NodeHandle &n, tf::Vector3 &can_location){
    geometry_msgs::Pose start_pose;
    start_pose.position.x = can_location.getX();
    start_pose.position.y = can_location.getY();
    start_pose.position.z = can_location.getZ();

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = (std::string) "block";
    modelstate.reference_frame = (std::string) "world";
    modelstate.pose = start_pose;

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;
    client.call(setmodelstate);
}


void set_table_location(ros::NodeHandle &n, tf::Vector3 &table_location){
    geometry_msgs::Pose start_pose;
    start_pose.position.x = table_location.getX();
    start_pose.position.y = table_location.getY();
    start_pose.position.z = table_location.getZ();

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = (std::string) "table_1";
    modelstate.reference_frame = (std::string) "world";
    modelstate.pose = start_pose;
    //    modelstate.twist = start_twist;

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;
    client.call(setmodelstate);
}

void reset_object_locations(ros::NodeHandle &n){
//    tf::Vector3 table_location(1.028586, -0.778717,0);
    tf::Vector3 table_location(1.028586, -0.778717,0);
    set_table_location(n, table_location);

    tf::Vector3 block_location(0.60, -0.4, 1.125);  
//    tf::Vector3 block_location(0.60, -0.4, 1.125);  
    set_object_location(n, block_location);
}


void move_pr2_to_starting_pos(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &start_pose2){
    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());  
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    
    robot_state::RobotState start_state(*group.getCurrentState());

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
    sleep(5.0);
    // When done with the path constraint be sure to clear it.
    group.clearPathConstraints();

}

double gaussian(double x, double height, double center,double variance){
//    std::cout << M_PI << std::endl;
    double a = 1/(sqrt(2*variance*M_PI));
    // std::cout << x ;
    // std::cout << " " ;
    // std::cout << height*a*exp(-( pow((x-center),2) / (2*variance) ) ) << std::endl;

    return height*a*exp(- ( pow((x-center),2) / (2*variance) ) ); 
}

std::vector<double> max_elements(int n, std::vector<tf::Vector3> f_s){
    std::vector<double> max_vector;
    double x_max_candidate = f_s[0].getX();
    double y_max_candidate = f_s[0].getY();
    double z_max_candidate = f_s[0].getZ();
        for (std::vector<int>::size_type i = 0; i < n; ++i){
            if (x_max_candidate < f_s[i].getX()){
                x_max_candidate = f_s[i].getX();
            }
            if (y_max_candidate < f_s[i].getY()){
                y_max_candidate = f_s[i].getY();
            }
            if (z_max_candidate < f_s[i].getZ()){
                z_max_candidate = f_s[i].getZ();
            }                        
        }
    max_vector.push_back(x_max_candidate);
    max_vector.push_back(y_max_candidate);    
    max_vector.push_back(z_max_candidate);
    return max_vector;
}

double getRandom_num(){
    // Random number generator where r is -1 < r < 1;
    std::srand(time(NULL));
    double r = 2*(((double) rand() / (RAND_MAX)) - 0.5);
    return r;    
}

double getRandom_percent(){
    // Random number generator where r is -1 < r < 1;
    std::srand(time(NULL));
    double r = ((double) rand() / (RAND_MAX));
    return r;    
}

std::vector<double> policy_search(double score, DMP_param &orig_dmp, DMP_param &perturbed_dmp_policy, tf::Vector3 &goal_offset){
    std::vector<double> policy;

    // Copy all of orig_dmp except the f(s)
    perturbed_dmp_policy.K = orig_dmp.K;
    perturbed_dmp_policy.D = orig_dmp.D;
    perturbed_dmp_policy.tau_demo = orig_dmp.tau_demo;
    perturbed_dmp_policy.alpha = orig_dmp.alpha;
    perturbed_dmp_policy.n_samples = orig_dmp.n_samples;              

    perturbed_dmp_policy.demo_t = orig_dmp.demo_t;
    perturbed_dmp_policy.demo_pos = orig_dmp.demo_pos;
    perturbed_dmp_policy.demo_vel = orig_dmp.demo_vel;
    perturbed_dmp_policy.demo_acel = orig_dmp.demo_acel;   


    // Count number of elements in f_s
    int n = 0;
    for(std::vector<double>::iterator s_i = orig_dmp.s.begin(); s_i != orig_dmp.s.end(); ++s_i) {
        n++;       
    }    

    std::vector<double> f_s_max = max_elements(n, orig_dmp.f_s);

    double height = 1.0;
    double variance = 0.02;
    double s_x_center = 0.8 + 0.2*getRandom_percent();
    double s_y_center = 0.8 + 0.2*getRandom_percent();
    double s_z_center = 0.8 + 0.2*getRandom_percent();        

    double x_delta = 0.01*getRandom_num()*f_s_max[0];
    double y_delta = 0.30*getRandom_num()*f_s_max[1];
    double z_delta = 0.15*getRandom_num()*f_s_max[2];

    for (std::vector<int>::size_type i = 0; i < n; ++i){
        double s_cur = orig_dmp.s[i];        
                

        double fs_x = orig_dmp.f_s[i].getX() + x_delta*(gaussian(s_cur, height, s_x_center, variance)/gaussian(s_x_center, height, s_x_center, variance));                
        double fs_y = orig_dmp.f_s[i].getY() + y_delta*(gaussian(s_cur, height, s_y_center, variance)/gaussian(s_y_center, height, s_y_center, variance));        
        double fs_z = orig_dmp.f_s[i].getZ() + z_delta*(gaussian(s_cur, height, s_z_center, variance)/gaussian(s_z_center, height, s_z_center, variance));

        std::cout << s_cur ;
        std::cout << " ";
        std::cout << fs_x;
        std::cout << " ";
        std::cout << fs_y;
        std::cout << " ";
        std::cout << fs_z << std::endl; 

        tf::Vector3 new_fs(fs_x, fs_y, fs_z);        
        perturbed_dmp_policy.f_s.push_back(new_fs);        
        perturbed_dmp_policy.s.push_back(s_cur);
    }            
    double add_x = 0.02*getRandom_num();
    double add_z = 0.05*getRandom_num();

    goal_offset.setX(goal_offset.getX()+add_x);
    goal_offset.setZ(goal_offset.getZ()+add_z);

    policy.push_back(s_x_center);
    policy.push_back(s_y_center);
    policy.push_back(s_z_center);
    policy.push_back(x_delta);
    policy.push_back(y_delta);
    policy.push_back(z_delta);    
    policy.push_back(add_x);    
    policy.push_back(add_z);            
    return policy;                        

}

// Initialize all to 0.
void init_policy(std::vector<double> policy){
    policy.push_back(0);
    policy.push_back(0);    
    policy.push_back(0);
    policy.push_back(0);    
    policy.push_back(0);
    policy.push_back(0);
    policy.push_back(0);
    policy.push_back(0);        
}


void print_policy_parameters(std::vector<double> policy){
    std::cout << "s_x_center";
    std::cout << " ";
    std::cout << policy[0] << std::endl;
    std::cout << "s_y_center";
    std::cout << " ";
    std::cout << policy[1] << std::endl;
    std::cout << "s_z_center";
    std::cout << " ";
    std::cout << policy[2] << std::endl;
    std::cout << "x_delta";
    std::cout << " ";
    std::cout << policy[3] << std::endl;
    std::cout << "y_delta";
    std::cout << " ";
    std::cout << policy[4] << std::endl;
    std::cout << "z_delta";
    std::cout << " ";
    std::cout << policy[5] << std::endl;
    std::cout << "add_x";
    std::cout << " ";
    std::cout << policy[6] << std::endl;
    std::cout << "add_z";
    std::cout << " ";
    std::cout << policy[7] << std::endl;    
}

int main(int argc, char **argv){
    ros::init (argc, argv, "dmp_pr2_execute");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher rvizMarkerPub; // Create Marker Publisher
    rvizMarkerPub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1000); // Advertise Marker Publisher

    DMP_param reaching_dmp;
    DEMO_traj reaching_demo_traj;


    // Initialize Pr2 desired starting pose    
     geometry_msgs::Pose start_pose2;
    start_pose2.orientation.x = 1.0;
    start_pose2.orientation.y = 0.0; 
    start_pose2.orientation.z = 0.0;
    start_pose2.orientation.w = 0.0;
    start_pose2.position.x = 0.55;//0.55;
    start_pose2.position.y = -0.15;//-0.05;
    start_pose2.position.z = 1.2;//0.8;



    reset_object_locations(n);  

    // MOVEIT
    // Initialize moveit
    moveit::planning_interface::MoveGroup group("right_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
    move_pr2_to_starting_pos(group, start_pose2);
    // //MOVEIT
                 
    reset_object_locations(n);
    sleep(15);

    // group;
    // start_pose2;
    // reaching_dmp;
    // reaching_demo_traj;
    
    DMP_param candidate_best_dmp;
    DMP_param dmp_to_try;
    std::vector<double> best_policy;
    init_policy(best_policy);



    learn_dmp(reaching_dmp, reaching_demo_traj); //Learn a dmp called reaching_dmp  using a reaching_demo_traj        
    tf::Vector3 demo_offset;
    demo_offset = obtain_demo_object_start_pos(DEMO_FILE_NAME , MARKER_ID_TO_REORIENT, reaching_demo_traj);

     tf::Vector3 goal_offset_pos(-0.05,0.06225,demo_offset.getZ()-.03); //goal offset    
//     tf::Vector3 goal_offset_pos(-0.05,0.06725,demo_offset.getZ()-.03); //goal offset    
//    tf::Vector3 goal_offset_pos(-0.05,0.04725,demo_offset.getZ()); 

    double score = 0;
    double new_score = 0;        
    for (std::vector<int>::size_type i_learn = 0; i_learn < 10; ++i_learn){
        ROS_INFO("Starting new trial. Iteration #");
        std::cout << i_learn << std::endl;

        Waypoints_traj des_waypoints; // Create new des_waypoints
        Waypoints_traj converted_des_waypoints;
        DMP_param perturbed_dmp_policy;
        std::vector<double> try_policy;

        // Compute Goal
        tf::Vector3 r_gripper_position(start_pose2.position.x, start_pose2.position.y, start_pose2.position.z);    
        //tf::Vector3 pr2_base_link_pos(0,0,0.051); //is the initialization of pr2. height offset it seems.
        tf::Vector3 pr2_base_link_pos(0,0,0.000575); //is the height of the base_link from the ground

        tf::Vector3 object_start_pos(0.6, -0.4, 1.105); // Initial Object Position
        
        //tf::Vector3 pr2_goal_location(object_start_pos - pr2_base_link_pos - r_gripper_position + goal_offset_pos); //relative position from the gripper.               
        tf::Vector3 pr2_goal_location(get_object_pos(n) - pr2_base_link_pos - r_gripper_position + goal_offset_pos); //relative position from the gripper.    

    
        // IMPORTANT!! when learning the dmp, convert pr2's frame to the dmp frame.
        tf::Vector3 compute_dmp_goal = pr2_traj_to_dmp_frame(pr2_goal_location);



        // DO_DMP policy search here.
        if (i_learn == 0){
            try_policy = best_policy;
            candidate_best_dmp.replace_DMP_with(reaching_dmp);
            try_policy = policy_search(score, candidate_best_dmp, perturbed_dmp_policy, goal_offset_pos);            
            //dmp_to_try.replace_DMP_with(perturbed_dmp_policy);
            dmp_to_try.replace_DMP_with(reaching_dmp);
        } else{
            try_policy = policy_search(score, candidate_best_dmp, perturbed_dmp_policy, goal_offset_pos); // Mutates perturbed_dmp_policy. Stores Policy
            dmp_to_try.replace_DMP_with(perturbed_dmp_policy);
        }

        plan_dmp_trajectory(dmp_to_try, des_waypoints, compute_dmp_goal); // plan a trajectory using a dmp. use dmp_goal as the new goal. mutate des_waypoints.


        // IMPORTANT!! all x-values need to be converted back to pr2_frame due to working with the kinnect
        convert_waypoints_to_pr2_axes(des_waypoints, converted_des_waypoints); 
 
        // Find the transform to move the points to the gripper
        tf::Vector3 first_waypoint_vector_offset(converted_des_waypoints.pos[0].getX(), converted_des_waypoints.pos[0].getY(), 
                                                                                        converted_des_waypoints.pos[0].getZ());
        tf::Transform translate_to_main_axis(tf::Quaternion(0,0,0,1), r_gripper_position - first_waypoint_vector_offset); // Create translation transform
        tf::Transform rotate_to_main_axis(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)); // Create rotate transform. Don't rotate.    

        // Count number of elements in waypoints 
        int n_waypoints = 0;
        for(std::vector<double>::iterator t_i = converted_des_waypoints.time.begin(); t_i != converted_des_waypoints.time.end(); ++t_i) {
            n_waypoints++;       
        }    
        std::vector<geometry_msgs::Pose> right_arm_waypoints;
        // Plot waypoints in rviz and push them to waypoints:
        for (std::vector<int>::size_type i = 0; i < n_waypoints; ++i){
            pub_dmp_markers(rvizMarkerPub, i, n_waypoints, converted_des_waypoints, translate_to_main_axis, rotate_to_main_axis);
            create_pr2_waypoints(i, converted_des_waypoints, right_arm_waypoints, translate_to_main_axis, rotate_to_main_axis);
        }


      // MOVEIT
      // Cartesian Paths
      moveit_msgs::RobotTrajectory trajectory;
      double fraction = group.computeCartesianPath(right_arm_waypoints,
                                                   0.01,  // eef_step
                                                   0.0,   // jump_threshold
                                                   trajectory);

      ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
            fraction * 100.0);    
      ROS_INFO("Executing plan (cartesian path)");
      moveit::planning_interface::MoveGroup::Plan plan;
      plan.trajectory_ = trajectory;
      group.execute(plan);
      // CARTESIAN PATHS
      // MOVEIT

      new_score = abs(object_start_pos.getX() - get_object_pos(n).getX()) + abs(object_start_pos.getY() - get_object_pos(n).getY()) + 10*(get_object_pos(n).getZ() - object_start_pos.getZ());  

      try_policy.push_back(new_score);
      print_policy_parameters(try_policy);
        
      ROS_INFO("This policy's score was ");
      std::cout << new_score << std::endl;
      sleep(3);
      if ((i_learn == 0) || (score < new_score)){
          ROS_INFO("It's a new better policy!");
          candidate_best_dmp.replace_DMP_with(dmp_to_try);
          best_policy = try_policy;
      }            

       move_pr2_to_starting_pos(group, start_pose2);                     
       reset_object_locations(n);
       sleep(5);       
    } 
    ROS_INFO("Finished exploring");    
    ROS_INFO("Best policy found was");
    print_policy_parameters(best_policy);


  
  sleep(15.0);
  ros::shutdown();  



}


        // tf::Transform translate_to_main_axis(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0) ); // Create translation transform
        // tf::Transform rotate_to_main_axis(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)); // Create rotate transform. Don't rotate.    

        // int n_waypoints = 0;
        // for(std::vector<double>::iterator t_i = des_waypoints.time.begin(); t_i != des_waypoints.time.end(); ++t_i) {
        //     n_waypoints++;       
        // }    
        
        // std::vector<geometry_msgs::Pose> right_arm_waypoints;
        // // Plot waypoints in rviz and push them to waypoints:
        // for (std::vector<int>::size_type i = 0; i < n_waypoints; ++i){
        //     pub_dmp_markers(rvizMarkerPub, i, n_waypoints, des_waypoints, translate_to_main_axis, rotate_to_main_axis);
        // }