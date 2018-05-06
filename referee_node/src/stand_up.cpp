#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "iostream"
using namespace std;
void pose_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  cout << "hallo" << endl;
  if (abs(msg->pose.pose.orientation.x) > 0.3 || abs(msg->pose.pose.orientation.y)> 0.3) {
     ROS_INFO("Ich liege");
  } 
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Position_listener");

  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("/odom", 10, pose_Callback);
  ros::spin();
  return 0;
}
