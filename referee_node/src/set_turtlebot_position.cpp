/*
 * set_turtlebot_position.cpp
 *
 *  Created on: Apr 24, 2018
 *  Author: David
 */

#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<gazebo_msgs/ModelState.h>

int main(int argc, char** argv)
{


	ros::init(argc, argv, "reset_position");
	ros::NodeHandle nh;

        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = (std::string) "turtlebot3";
        modelstate.pose.position.x = 1;
        modelstate.pose.position.y = 0;
        modelstate.pose.position.z = 10;
        modelstate.reference_frame = (std::string) "world";
	ros::Publisher publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
        sleep(1);
        int counter = 0;
        while (counter < 100) 
        {
            publisher.publish(modelstate);
            counter++;
        }
  
}
