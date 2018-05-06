/*
 * Stopper.h
 *
 *  Created on: Apr 14, 2018
 *      Author: yashuai
 */

#ifndef STOPPER_H_
#define STOPPER_H_

#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

class Stopper {
public:
	// Tunable parameters
	const static double FORWARD_SPEED_MPS= 0.5;
	const static double MIN_SCAN_ANGLE_RAD = 30.0 / 180.0 * M_PI;
	const static double MAX_SCAN_ANGLE_RAD = 360.0 / 180.0 * M_PI;
	const static float MIN_PROXIMITY_RANGE_M = 0.5;

public:
	Stopper();
	virtual ~Stopper();
	void startMoving();

private:
	ros::NodeHandle nh;
	ros::Publisher cmdPub;		// Publisher to the robot's velocity command topic
	ros::Subscriber laserSub;	// Subscriber to the robot's laser scan topic
	bool keepMoving;			// Indicates whether the robot should continue moving

	void move_forward();
	void turn_left();
	void stop_move();
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
};

#endif /* STOPPER_H_ */
