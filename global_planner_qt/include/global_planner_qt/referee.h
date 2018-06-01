#ifndef REFEREE_H
#define REFEREE_H

#include "ros/ros.h"
#include "referee_node/referee_machines.h"
#include "referee_node/referee_resources.h"

namespace global_planner {
class Referee
{
private:
  ros::NodeHandle nh;
  ros::Subscriber machine_sub;
  ros::Subscriber resource_sub;

  bool machines_updated;
  bool resources_updated;
public:
  Referee();
  void update_referee_info();
private:
  void machine_pose_callback(const referee_node::referee_machines::ConstPtr &msg);
  void resource_pose_callback(const referee_node::referee_resources::ConstPtr &msg);
};

} // namespace global_planner

#endif // REFEREE_H
