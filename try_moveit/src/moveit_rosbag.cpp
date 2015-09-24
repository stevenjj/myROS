#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <visualization_msgs/Marker.h>

#include <boost/foreach.hpp>

#include <string.h>
#include <iostream>

#define foreach BOOST_FOREACH


int main(int argc, char **argv){

    ROS_INFO("Opening Bag");
    rosbag::Bag bag;
    bag.open("trajectory_bag.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("visualization_marker"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
//    std::cout << "hello world!" << std::endl;
    
    foreach(rosbag::MessageInstance const m, view)
    {
        //std::cout << m.getTopic() << std::endl;
    ROS_INFO("Inside bag!");
        //geometry_msgs::Pose::ConstPtr p = m.instantiate<geometry_msgs::Pose>();
        visualization_msgs::Marker::ConstPtr p = m.instantiate<visualization_msgs::Marker>();
        std::cout << m.getDataType() << std::endl;
        std::cout << p->id << std::endl;
        std::cout << p->pose.position.x << std::endl;
        std::cout << p->pose.position.y << std::endl;
        std::cout << p->pose.position.z << std::endl;
        std::cout << p->pose.orientation.x << std::endl;
        std::cout << p->pose.orientation.y << std::endl;
        std::cout << p->pose.orientation.z << std::endl;
        std::cout << p->pose.orientation.w << std::endl;

    }
    ROS_INFO("Closing bag");
    
    bag.close();

}