#include "../include/global_planner_qt/drop.h"
#include "../include/global_planner_qt/param.h"
#include "gazebo_msgs/ModelState.h"
#include "robot_navigation/Attach.h"

namespace global_planner {
int Drop::current_publish_id = 0;
Drop::Drop() : Resource()
{
  found_resources = false;

  publish_machine = nh.advertise<robot_navigation::got_resource>("/roboter/resource", 1);
  client = nh.serviceClient<robot_navigation::Attach>("/link_attacher_node/detach");

  sub_resource_position = nh.subscribe("/gazebo/model_states", 1, &Drop::resource_pose_Callback, this);
  sub = nh.subscribe("/gazebo/link_states", 1, &Drop::pose_Callback, this);
}

/*
 *Method drops the nearest resource when the method is called
 */
void Drop::pose_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
  if(!found_roboter_link)
  {
    link_info.name = msg->name;
    link_info.pose = msg->pose;
    found_roboter_link = true;
  }
}

/*
 *This function must be called before pose_callback because of resource update
 *updates the resources
 */
void Drop::resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if (!found_resources)
    {
        local_resources.clear();
        long unsigned int size = msg->name.size();

        //std::cout << "msg->name.size(): " << msg->name.size() << "\n";
        int i;
        for(i = 0; i < (int) size; i++)     //;
        {
            if(msg->name[i].find("Resource") != std::string::npos)
            {       
                Resource_t new_resource;
                new_resource.name = msg->name[i];
                new_resource.x = msg->pose[i].position.x;
                new_resource.y = msg->pose[i].position.y;
                ROS_ERROR("%s", new_resource.name.c_str());
                local_resources.push_back(new_resource);
            }
        }
        found_resources = true;
    }
}

/**
 * callbacks get called once, while grabbed is true
 */
void Drop::drop_resource()
{
  //Wait until resources are found
  while(ok())
  {
    if(found_resources && found_roboter_link)
    {
       int i = 0; //Gripper-Link ID
       int nearest_resource_index = 0;
       double smallest_distance = std::numeric_limits<double>::infinity();

       //Find Gripper
       std::string link = roboter_name + "::gripper_link";
       while (link_info.name[i] != link)
       {
         i++;
       }

       //Find smallest distance between gripper and resources => Get nearest resource
       int resource_size = (int)local_resources.size();
       if(resource_size == 0)
       {
         return;
       }
       for(int k = 0; k < resource_size; k++) // find nearest resource
       {
           double temp_distance = sqrt( pow(link_info.pose[i].position.x - local_resources[k].x, 2) + pow(link_info.pose[i].position.y - local_resources[k].y, 2) );;
           if(temp_distance < smallest_distance)
           {
               smallest_distance = temp_distance;
               nearest_resource_index = k;
           }
       }

       //Publish Messages
       robot_navigation::got_resource resource_msg;
       resource_msg.roboter_name = roboter_name;
       resource_msg.resource_name = local_resources.at(nearest_resource_index).name;

       //Detach Resource
       robot_navigation::Attach attach_srv;
       attach_srv.request.model_name_1 = local_resources.at(nearest_resource_index).name;
       attach_srv.request.link_name_1 = "link";
       attach_srv.request.model_name_2 = roboter_name;
       attach_srv.request.link_name_2 = "gripper_link";

       //Set Resource blocked or unblocked
       if(machine_id != -1)
       {
         resources[nearest_resource_index].blocked = false;
         resource_msg.resource_blocked = false;
       }
       else
       {
         resource_msg.resource_blocked = true;
       }

       sleep(1);
       publish_machine.publish(resource_msg);
       client.call(attach_srv);

       //Reset drop
       grabbed = false;
       machine_id = -1;
       break;
    }

    Duration(0.1).sleep();
    spinOnce();
  }

  current_resource = "";
}

}

