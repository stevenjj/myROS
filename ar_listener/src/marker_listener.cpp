#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void markerCallback(const ar_track_alvar::AlvarMarkers& msg)

{
  ROS_INFO("I heard: [%i]", msg.markers.size()   );
  if (msg.markers.size() > 0) {

    for (int i = 0; i < msg.markers.size(); i++){
      ROS_INFO("I see ID: %i", msg.markers[i].id);
      ROS_INFO("position x: %0.2f", msg.markers[i].pose.pose.position.x);
    }

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ar_pose_marker", 0, markerCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}