#ifndef GRAB_H
#define GRAB_H

#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "robot_navigation/got_resource.h"
#include "resource.h"

//#include "gazebo_msgs/ModelStates.h"
//#include "global_planner_qt/got_resource.h"

using namespace ros;

namespace global_planner {
class Grab : public Resource
{
private:
  ros::NodeHandle nh;
  ros::Publisher publisher;
  ros::Subscriber sub;
  ros::Subscriber sub_resource_position;

  bool found_resources;
  bool found_roboter_link;

  std::vector<Resource::Resource_t> local_resources;
  gazebo_msgs::LinkStates link_info;

public:
  Grab();
  void grab_resource();
private:
  void pose_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg);
  void resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};
}


#endif // GRAB_H
