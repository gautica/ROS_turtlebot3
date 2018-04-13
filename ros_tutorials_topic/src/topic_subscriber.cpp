#include<ros/ros.h>
// The header file is automatically created wenn building the package.
#include<ros_tutorials_topic/MsgTutorial.h>

/**
 * Message callback function. This is a function is called when a topic
 * message named 'ros_tutorial_msg' is received.'
 */
void msgCallback(const ros_tutorials_topic::MsgTutorial msg)
{
	ROS_INFO("recieve message: %d", msg.stamp.sec);
	ROS_INFO("recieve message: %d", msg.stamp.nsec);
	ROS_INFO("recieve message: %d", msg.data);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "topic_subscriber");	// Initialize Node Name

	ros::NodeHandle nh;
	
	// Declare subscriber using MsgTutorial message file
	ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 100, msgCallback);

	// A function for calling a callback function, waiting for a message to be
	// recieved, and executing a callback function when it is recieved
	ros::spin();
}
