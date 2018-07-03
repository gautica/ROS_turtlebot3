#ifndef DROP_H
#define DROP_H

#include <ros/ros.h>
#include "gazebo_msgs/ModelStates.h"
#include "robot_navigation/got_resource.h"
#include "gazebo_msgs/LinkStates.h"
#include "resource.h"

using namespace std;
using namespace ros;

namespace global_planner {
class Drop : public Resource
{
public:
  static int current_publish_id;
private:
  ros::NodeHandle nh;
  ros::Publisher publisher;
  ros::Subscriber sub;
  ros::Subscriber sub_resource_position;
  ros::Publisher publish_machine;

  bool found_resources;
  bool found_roboter_link;

  std::vector<Resource::Resource_t> local_resources;
  gazebo_msgs::LinkStates link_info;

public:
  Drop();
  void drop_resource();
private:
  void pose_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg);
  //void resource_callback(const global_planner_qt::got_resource::ConstPtr& msg);
  void resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};
}


#endif // DROP_H
