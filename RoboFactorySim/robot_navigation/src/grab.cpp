#include "../include/global_planner_qt/grab.h"
#include "../include/global_planner_qt/param.h"
#include "gazebo_msgs/ModelState.h"
#include <stdlib.h>
#include <iostream>
#include "robot_navigation/Attach.h"

namespace global_planner {

Grab::Grab() : Resource()
{
  found_resources = false;

  publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
  sub_resource_position = nh.subscribe("/gazebo/model_states", 1, &Grab::resource_pose_Callback, this);
  sub = nh.subscribe("/gazebo/link_states", 1, &Grab::pose_Callback, this);
}

void Grab::pose_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    link_info.name = msg->name;
    link_info.pose = msg->pose;
}

void Grab::resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if (!found_resources)
    {
        local_resources.clear();
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

                local_resources.push_back(new_resource);
            }
        }
        found_resources = true; 
    }
}

void Grab::grab_resource()
{
    while(ok())
    {
        if(found_resources)
        {
            int i = 0;
            int nearest_resource_index = 0;

            int resource_size = (int)local_resources.size();
            double smallest_distance = std::numeric_limits<double>::infinity();
            if(resource_size <= 0)
            {
                return;
            }
            std::string link = roboter_name + "::gripper_link";
            std::cout << "link: " << link << "\n";
            while (link_info.name[i].compare(link) != 0)
            {
              i++;
           }

            for(int k = 0; k < resource_size; k++)
            {
                double temp_distance = sqrt( pow(link_info.pose[i].position.x - local_resources[k].x, 2) + pow(link_info.pose[i].position.y - local_resources[k].y, 2) );;
                if(local_resources[k].name.find(will_grab.c_str()) != std::string::npos && temp_distance < smallest_distance)
                {
                    smallest_distance = temp_distance;
                    nearest_resource_index = k;
                }
            }
            sleep(1);
            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = (std::string) local_resources.at(nearest_resource_index).name;
            modelstate.pose.position.x = link_info.pose[i].position.x;
            modelstate.pose.position.y = link_info.pose[i].position.y;
            modelstate.pose.position.z = 0.03;
            modelstate.reference_frame = (std::string) "world";
            publisher.publish(modelstate);

            resources[nearest_resource_index].blocked = true;

            //Ros Service Call
            ros::ServiceClient client = nh.serviceClient<robot_navigation::Attach>("/link_attacher_node/attach");
            robot_navigation::Attach attach_srv;
            attach_srv.request.model_name_1 = local_resources.at(nearest_resource_index).name;
            attach_srv.request.link_name_1 = "link";
            attach_srv.request.model_name_2 = roboter_name;
            attach_srv.request.link_name_2 = "gripper_link";

            if (client.call(attach_srv))
            {
                ROS_ERROR("Attached");
            }
            else
            {
                ROS_ERROR("Attach FAILED");
            }

            current_resource = link_info.name[i];
            break;
        }

        Duration(0.01).sleep();
        ros::spinOnce();
    }
}

}


