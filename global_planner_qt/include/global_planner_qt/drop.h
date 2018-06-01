#ifndef DROP_H
#define DROP_H

#include <ros/ros.h>
#include "gazebo_msgs/ModelStates.h"
#include "global_planner_qt/got_resource.h"
#include "gazebo_msgs/LinkStates.h"
#include "resource.h"

namespace global_planner {
class Drop : public Resource
{
public:
  static int current_publish_id;
private:
  /**
  ros::NodeHandle nh;
  ros::Subscriber sub_resource;
  ros::Subscriber sub_resource_position;

  int sub_counter;
*/
  ros::NodeHandle nh;
  ros::Publisher publisher;
  ros::Subscriber sub;
  ros::Subscriber sub_resource_position;
  
  bool found_resources;
  std::vector<Resource::Resource_t> resources;

  //int resources_in_simulation;

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
