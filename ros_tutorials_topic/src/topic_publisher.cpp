#include<ros/ros.h>
#include"ros_tutorials_topic/MsgTutorial.h"
// The header file is automatically creaed when building package

int main(int argc, char** argv)		// Node Main Function
{
	ros::init(argc, argv, "topic_publisher");	// Initializes Node Name
	ros::NodeHandle nh;	// Node handle declaration for communication with ROS system
	
	/*
	 * Declare publisher, create publisher 'ros_tutorial_pub' using the 
	 * 'MsgTutoral.msg' file from 'ros_tutorial_topic' package. The topic
         *  name is 'ros_tutorial_msg' and the size of the of the publisher
	 *  queue is set to 100.
	 */
	ros::Publisher ros_tutorial_pub = 
				nh.advertise<ros_tutorials_topic::MsgTutorial>("ros_tutorial_msg", 100);
	
	// Set the loop period. '10' refers to 10 Hz and main loop repeats at 0.1 seconds intercals
	ros::Rate loop_rate(1);

	// Declare message 'msg' in 'MsgTutorial' message file format
	ros_tutorials_topic::MsgTutorial msg;

	int count = 0; 		// Variable to be used in message

	while (ros::ok()) {
		msg.stamp = ros::Time::now();	// save current time in stamp of 'msg'
		msg.data = count;		// ...
		
		ROS_INFO("send message = %d", msg.stamp.sec);	// print the 'stamp.sec' message
		ROS_INFO("send message = %d", msg.stamp.nsec);
		ROS_INFO("send message = %d", msg.data);

		ros_tutorial_pub.publish(msg);		// publish message
		loop_rate.sleep();		// goes to sleep according to the loop rate defined above
		
		count++;
	}

	return 0;	
}
