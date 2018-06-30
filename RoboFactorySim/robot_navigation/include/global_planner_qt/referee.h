#ifndef REFEREE_H
#define REFEREE_H

#include "ros/ros.h"
#include "referee_node/referee_machines.h"
#include "referee_node/referee_resources.h"

using namespace std;

namespace global_planner {
class Referee
{
public:
  struct Machine_Struct
  {
      int id;
      float pos_x;
      float pos_y;
      float time_working;
      float radius;
      float distance_to_roboter;
      bool is_working;
      bool is_publishing;
      string wanted_resource[4];
      string product[4];
      float deposition_x;
      float deposition_y;
      float deposition_z;
      int apply_force_orientation; //0: x, 1: y, 2: -x, 3:-y
  };

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
