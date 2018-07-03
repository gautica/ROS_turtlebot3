#include "../include/global_planner_qt/referee.h"
#include "../include/global_planner_qt/param.h"
#include "robot_navigation/got_resource.h"

namespace global_planner {

Referee::Referee()
{
  machine_sub = nh.subscribe("referee_machines", 10, &Referee::machine_pose_callback, this);
  resource_sub = nh.subscribe("referee_resources", 10, &Referee::resource_pose_callback, this);

  machines_updated = false;
  resources_updated = false;
}

void Referee::machine_pose_callback(const referee_node::referee_machines::ConstPtr &msg)
{
  machines.clear();

  global_planner::Referee::Machine_Struct machine;

  for (int i = 0; i < msg->machines.size(); i++) {
    machine.id = msg->machines[i].machine_id;
    machine.pos_x = msg->machines[i].pos_x;
    machine.pos_y = msg->machines[i].pos_y;
    machine.radius = msg->machines[i].radius;
    machine.is_working = msg->machines[i].is_working;

    machines.push_back(machine);
  }

  machines_updated = true;
}

void Referee::resource_pose_callback(const referee_node::referee_resources::ConstPtr &msg)
{

  resources.clear();
  for(int i = 0; i < msg->resources.size(); i++)
  {
      resources.push_back(msg->resources[i]);
  }
  resources_updated = true;
}

void Referee::update_referee_info()
{
  ROS_INFO("Updating machines and resources ...");
  while (!machines_updated || !resources_updated) {
    ros::spinOnce();
    sleep(0.5);
  }
  ROS_INFO("Machines and Resources are updated");
}

}   // namespace global_planner
