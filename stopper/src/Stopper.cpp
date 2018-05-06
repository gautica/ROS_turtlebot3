/*
 * Stopper.cpp
 *
 *  Created on: Apr 14, 2018
 *      Author: yashuai
 */

#include "Stopper.h"
#include <geometry_msgs/Twist.h>

Stopper::Stopper() {
	// TODO Auto-generated constructor stub
	keepMoving = true;

	// Advertise a new publisher for the robot's velocity command topic
	cmdPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	// Subscribe to the simulated robot's laser scan topic
	laserSub = nh.subscribe("/scan", 1, &Stopper::scan_callback, this);
}

Stopper::~Stopper() {
	// TODO Auto-generated destructor stub
}

void Stopper::move_forward()
{
	geometry_msgs::Twist msg;

	msg.linear.x = FORWARD_SPEED_MPS;

	cmdPub.publish(msg);
}

void Stopper::stop_move()
{
	geometry_msgs::Twist msg;

	msg.linear.x = 0.0;

	cmdPub.publish(msg);
}

void Stopper::turn_left()
{
	geometry_msgs::Twist msg;

	// Turn left 90 greed
	msg.angular.z = M_PI / 4;

	cmdPub.publish(msg);

	keepMoving = true;
}

void Stopper::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
	// Find the closest range between the defined minimum and maximum angles
	int minIndex = (int) ceil((MIN_SCAN_ANGLE_RAD - scanMsg->angle_min) / scanMsg->angle_increment);
	int maxIndex = (int) ceil(scanMsg->angle_max / scanMsg->angle_increment);

	float closest_range = scanMsg->ranges[0];

	for (int i = 0; i < minIndex; i++)
	{
		if (scanMsg->ranges[i] < closest_range)
		{
			closest_range = scanMsg->ranges[i];
		}

		if (scanMsg->ranges[maxIndex - i - 2] < closest_range) {
			closest_range = scanMsg->ranges[maxIndex - i - 2];
		}

	}

	ROS_INFO_STREAM("Closest Range: "  << closest_range);

	if (closest_range < MIN_PROXIMITY_RANGE_M)
	{
		ROS_INFO("Turn Left");
		keepMoving = false;
	}
}

void Stopper::startMoving()
{
	ros::Rate loop_rate(10);

	ROS_INFO("Starting moving ...");
	int count  = 0;
  while (ros::ok() && count < 300)
	{
		if (keepMoving) {
			move_forward();
		} else {
			turn_left();
		}
		ros::spinOnce();	// Need to call this function to allow ros to process incoming messages
		loop_rate.sleep();

		count++;
	}

	stop_move();
	ros::spinOnce();

}

