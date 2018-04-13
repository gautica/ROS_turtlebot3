#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<ros_tutorials_action/FibonacciAction.h>

class FibonacciALG
{
protected:
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<ros_tutorials_action::FibonacciAction> as;
	std::string action_name;
	
	// Declare the action feedback and the result to Publish
	ros_tutorials_action::FibonacciFeedback feedback;
	ros_tutorials_action::FibonacciResult result;

public:
	FibonacciALG(std::string name) 
	: as(nh, name, boost::bind(&FibonacciALG::execute, this, _1), false),
	  action_name(name)
	{
		as.start();
	}
	~FibonacciALG(void) {}

	// A function that recieves an action goals message and performs a specified action
	void execute(const ros_tutorials_action::FibonacciGoalConstPtr &goal)
	{
		ros::Rate loop_rate(1);		// 1Hz
		bool is_success = true; 	// store success or failure of an action

		// Set Fibonacci sequence initializition
		feedback.sequence.clear();
		feedback.sequence.push_back(0);
		feedback.sequence.push_back(1);

		// Notify the user of action name, goal, initial two values of Fibonacci sequence
		ROS_INFO("%s: Executing and creating Fibonacci sequence of order %d with seed %d, %d",
			  action_name.c_str(), goal->order, feedback.sequence[0], feedback.sequence[1]);

		// Action content
		for (int i = 1; i <= goal->order; i++)
		{
  			// Confirm action cancellation from action client
			if (as.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name.c_str());
				as.setPreempted();	// Action cancellation
				is_success = false;
				break;
			}
			
			// Calculate Fibonacci number
			feedback.sequence.push_back(feedback.sequence[i] + feedback.sequence[i-1]);
			// Publish Feedback
			as.publishFeedback(feedback);
			loop_rate.sleep();
		}

		if (is_success)
		{
			result.sequence = feedback.sequence;
			ROS_INFO("%s: Succeeded", action_name.c_str());
			as.setSucceeded(result);
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_server");
	FibonacciALG fibonacci("ros_tutorial_action");
	ros::spin();		// Wait to receive action goal

	return 0;
}
