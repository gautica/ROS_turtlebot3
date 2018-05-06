/*
 * set_turtlebot_position.cpp
 *
 *  Created on: Apr 24, 2018
 *  Author: David
 */

// This code despawns any model in gazebo, just fill the name of the model in deleted model
#include<ros/ros.h>
#include<gazebo_msgs/DeleteModel.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "delete_Model");
	ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
        gazebo_msgs::DeleteModel deleted_model;
        deleted_model.request.model_name= (std::string) "turtlebot3_house"; 
        client.call(deleted_model);
}
