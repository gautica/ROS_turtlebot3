#include "../include/global_planner_qt/grab.h"
#include "../include/global_planner_qt/param.h"

namespace global_planner {

bool Grab::grabbed = false;

Grab::Grab()
{
  publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
  sub = nh.subscribe("/gazebo/link_states", 1, &Grab::pose_Callback, this);

  sub_resource = nh.subscribe("/roboter/resource", 10, &Grab::resource_callback, this);
  sub_resource_position = nh.subscribe("/gazebo/model_states", 10, &Grab::resource_pose_Callback, this);

  sub_counter = 0;
  current_publish_id = 0;
  found_ressources = false;
  resources_in_simulation = 3;
}

void Grab::pose_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    int i = 0;
    while (msg->name[i] != "turtlebot3_burger::gripper_link")
    {
      i++;
      std::cout << "i: " << i << std::endl;
    }
    ROS_ERROR("out of while ...");
   //float distance = sqrt(pow(msg->pose[i].position.x - -2.6, 2) + pow(msg->pose[i].position.y - -2.8, 2));

     if (!grabbed)
      {
        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = (std::string) "Resource_White_2";
        modelstate.pose.position.x = msg->pose[i].position.x;
        modelstate.pose.position.y = msg->pose[i].position.y;
        modelstate.pose.position.z = 0.03;
        modelstate.reference_frame = (std::string) "world";

       float time = 0;
       while(time < 0.3)
       {
          publisher.publish(modelstate);
          ros::Duration(0.05).sleep();
          time += 0.05;
       }
       grabbed = true;
       ROS_ERROR("debug1");
      }

}

void Grab::resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if(!found_ressources)
    {
        //Subscriber
        int i = 0;
        int found_obj = 1;
        while(found_obj < 4)
        {
            std::ostringstream convert;
            std::string resource_name = std::string("Resource_White_");

            convert << (found_obj);
            resource_name += convert.str();

            if(msg->name[i] == resource_name)
            {
                resources[found_obj - 1].x = msg->pose[i].position.x;
                resources[found_obj - 1].y = msg->pose[i].position.y;
                resources[found_obj - 1].name = msg->name[i];
                found_obj++;
            }
            i++;
        }

        found_ressources = true;
    }
}

void Grab::resource_callback(const global_planner_qt::got_resource::ConstPtr& msg)
{
    if(sub_counter != msg->id)
    {
        current_resource = msg->resource_name;
        grabbed = msg->got_r;
        sub_counter = msg->id;
        resources_in_simulation--;
    }

}

void Grab::grab_resource()
{
  while (!grabbed) {
    ros::spinOnce();
  }
}
}


