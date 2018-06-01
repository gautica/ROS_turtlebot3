#include "../include/global_planner_qt/grab.h"
#include "../include/global_planner_qt/param.h"
#include "gazebo_msgs/ModelState.h"
#include <stdlib.h>
#include <iostream>

namespace global_planner {

Grab::Grab() : Resource()
{
  publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
  sub = nh.subscribe("/gazebo/link_states", 1, &Grab::pose_Callback, this);
  sub_resource_position = nh.subscribe("/gazebo/model_states", 10, &Grab::resource_pose_Callback, this);
  found_resources = false;
}

void Grab::pose_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    if(!found_resources)
    {
        return;
    }
    
    int i = 0;
    int nearest_resource_index = 0;
    
    int resource_size = (int)resources.size();
    double smallest_distance = std::numeric_limits<double>::infinity();
    if(resource_size <= 0)
    {
        return;
    }
    
    while (msg->name[i] != "turtlebot3_burger::gripper_link")
    {
      i++;
      std::cout << "i: " << i << std::endl;
    }
    
    for(int k = 0; k < resource_size; k++)
    {
        double temp_distance = sqrt( pow(msg->pose[i].position.x - resources[k].x, 2) + pow(msg->pose[i].position.y - resources[k].y, 2) );;
        if(temp_distance < smallest_distance)
        {
            smallest_distance = temp_distance;
            nearest_resource_index = k;
        }
    }
    
    if (!grabbed && found_resources)
    {
        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = (std::string) resources.at(nearest_resource_index).name;
        std::cout << "resources[j].name: " << resources.at(nearest_resource_index).name.c_str() << "\n";
        modelstate.pose.position.x = msg->pose[i].position.x;
        modelstate.pose.position.y = msg->pose[i].position.y;
        modelstate.pose.position.z = 0.03;
        modelstate.reference_frame = (std::string) "world";

        float time = 0;
        while(time < 0.5)
        {
            publisher.publish(modelstate);
            ros::Duration(0.05).sleep();
            time += 0.05;
        }

        current_resource = msg->name[i];
        ROS_ERROR("Grab: %s", current_resource.c_str());
        grabbed = true;
    }   
}

void Grab::resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if (!found_resources)
    {
        resources.clear();
        long unsigned int size = msg->name.size();
        ROS_ERROR("%d", (int) size);

        int i;
        for(i = 0; i < (int) size; i++)
        {
            if(msg->name[i].find("Resource") != std::string::npos)
            {       
                Resource_t new_resource;
                new_resource.name = msg->name[i];
                new_resource.x = msg->pose[i].position.x;
                new_resource.y = msg->pose[i].position.y;

                ROS_ERROR("%s", new_resource.name.c_str());

                resources.push_back(new_resource);
            }
        }
        found_resources = true; 
    }
}

void Grab::grab_resource()
{
  while(!grabbed) {
    ros::spinOnce();
  }
}

}


