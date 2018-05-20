#ifndef GRAB_H
#define GRAB_H

#include <string>
#include <stdlib.h>
#include <iostream>

#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "global_planner_qt/got_resource.h"

namespace global_planner {
class Grab
{
  struct Resource
{
    std::string name;
    float x;
    float y;
    float distance;
};

public:
  static bool grabbed;

private:
  ros::NodeHandle nh;
  ros::Publisher publisher;
  ros::Subscriber sub;
  ros::Subscriber sub_resource;
  ros::Subscriber sub_resource_position;

  int sub_counter;

  Resource resources[3];
  std::string current_resource;
  int current_publish_id;
  bool found_ressources;
  int resources_in_simulation;

public:
  Grab();

  void grab_resource();
private:
  void pose_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg);
  void resource_callback(const global_planner_qt::got_resource::ConstPtr& msg);
  void resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};
}


#endif // GRAB_H
