#include "../include/global_planner_qt/drop.h"
#include "../include/global_planner_qt/param.h"
#include "global_planner_qt/got_resource.h"
#include "gazebo_msgs/ModelState.h"
namespace global_planner {
int Drop::current_publish_id = 0;
Drop::Drop() : Resource()
{
  publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
  sub = nh.subscribe("/gazebo/link_states", 1, &Drop::pose_Callback, this);
  sub_resource_position = nh.subscribe("/gazebo/model_states", 10, &Drop::resource_pose_Callback, this);
  found_resources = false;
}

/*
 *Method drops the nearest resource when the method is called
 */
void Drop::pose_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    if(!found_resources) // resources are not updated yet
    {
        return;
    }
    int i = 0; // gripper linker
    int s = 0; // base linker
    int nearest_resource_index = 0;

    int resource_size = (int)resources.size();
    double smallest_distance = std::numeric_limits<double>::infinity();

    if(resource_size <= 0) // no resources in simulation
    {
        return; 
    }

    while (msg->name[i] != "turtlebot3_burger::gripper_link")
    {
      i++;
      std::cout << "i: " << i << std::endl;
    }
    while (msg->name[s] != "turtlebot3_burger::base_footprint")
    {
      s++;
      std::cout << "i: " << i << std::endl;
    }
    float vec_x = msg->pose[i].position.x - msg->pose[s].position.x; //for front of roboter
    float vec_y = msg->pose[i].position.y - msg->pose[s].position.y;
    
    
    for(int k = 0; k < resource_size; k++) // find nearest resource
    {
        double temp_distance = sqrt( pow(msg->pose[i].position.x - resources[k].x, 2) + pow(msg->pose[i].position.y - resources[k].y, 2) );;
        if(temp_distance < smallest_distance)
        {
            smallest_distance = temp_distance;
            nearest_resource_index = k;
        }
    }

    if (grabbed) // if grabbed is true, set resource in front of robot
    {
        current_publish_id++;

        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = (std::string) resources.at(nearest_resource_index).name;
        std::cout << "resources[j].name: " << resources.at(nearest_resource_index).name.c_str() << "\n";
        modelstate.pose.position.x = vec_x * 2 + msg->pose[s].position.x ;
        modelstate.pose.position.y = vec_y * 2 + msg->pose[s].position.y;
        modelstate.pose.position.z = 0.03;
        modelstate.reference_frame = (std::string) "world";
        ros::Publisher publish_machine = nh.advertise<global_planner_qt::got_resource>("/roboter/resource", 100);
        global_planner_qt::got_resource resource_msg;
        resource_msg.resource_name = "Resource_White";
        resource_msg.id = current_publish_id;
        resource_msg.object_name = resources.at(nearest_resource_index).name;

        float time = 0;
        while(time < 0.5)
        {
            publish_machine.publish(resource_msg);
            publisher.publish(modelstate);
            ros::Duration(0.05).sleep();
            time += 0.05;
        }

        current_resource = msg->name[i];
        ROS_ERROR("Drop: %s", current_resource.c_str());
        grabbed = false;
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
        resources.clear();
        long unsigned int size = msg->name.size();
        ROS_ERROR("%d", (int) size);
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
                resources.push_back(new_resource);
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
  while(grabbed) {
    ros::spinOnce(); 
  }
}

}

