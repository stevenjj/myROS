#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

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
    std::cout << "hello world!" << std::endl;


    
    foreach(rosbag::MessageInstance const m, view)
    {
        std::cout << m.getTopic() << std::endl;
    ROS_INFO("Inside bag!");

//        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
//        if (s != NULL){
//            std::cout << s->data << std::endl;
//        }
//            ASSERT_EQ(s->data, std::string("foo"));

//        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//        if (i != NULL)
 //           ASSERT_EQ(i->data, 42);

    }
    ROS_INFO("Closing bag");
    
    bag.close();

}