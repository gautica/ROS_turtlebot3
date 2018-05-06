#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void pose_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Turtlebot at position x: [%f] y: [%f] z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  sleep(3);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Position_listener");

  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("/odom", 10, pose_Callback);

  ros::spin();

  return 0;
}

