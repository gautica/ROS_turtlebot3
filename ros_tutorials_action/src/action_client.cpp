#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/client/terminal_state.h>	// Action Goal Status Header File

#include"ros_tutorials_action/FibonacciAction.h"	// FibonacciAction Header File


int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_clinet");

	// Action Client Declaration (Action Name: ros_tutorial_action)
	actionlib::SimpleActionClient<ros_tutorials_action::FibonacciAction> ros_tutorial_ac(
	"ros_tutorial_action", true);

	ROS_INFO("Waiting for action server to start");
	ros_tutorial_ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");
	ros_tutorials_action::FibonacciGoal goal; 	// Declare Action Goal
	goal.order = 20;	// Set Action Goal (Process the Fibonacci sequence 20 times)
	ros_tutorial_ac.sendGoal(goal);		// Transmit Action Goal


	// Set action time limit (set to 30 seconds)
	bool finished_before_timeout = ros_tutorial_ac.waitForResult(ros::Duration(30));

	// Process when action results are recieved within the time limit for achieving the action goal
	if (finished_before_timeout)
	{
		// Recieve action target status value and display on screen
		actionlib::SimpleClientGoalState state = ros_tutorial_ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finished before time out.");
	}

	return 0;
}
