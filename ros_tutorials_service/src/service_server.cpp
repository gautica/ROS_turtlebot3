#include<ros/ros.h>

// Automatically created after build
#include"ros_tutorials_service/SrvTutorial.h"

#define PLUS	1
#define MINUS	2
#define MUL		3

int g_operator = 1;
// The below process is performed when is a service request
// The service request is declared as 'req', and the service response is declared as 'res'
bool calculation(ros_tutorials_service::SrvTutorial::Request &req,
		 ros_tutorials_service::SrvTutorial::Response &res)
{
	switch (g_operator) {
		case PLUS:
			res.result = req.req1 + req.req2;
			break;
		case MINUS:
			res.result = req.req1 - req.req2;
			break;
		case MUL:
			res.result = req.req1 * req.req2;
			break;
		default:
			res.result = req.req1 + req.req2;
	}

	ROS_INFO("request: x=%ld,  y=%ld", (long int)req.req1, (long int) req.req2);
	ROS_INFO("sending back response: %ld", (long int)res.result);

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "service_server");
	ros::NodeHandle nh;

	nh.setParam("calculation_method", PLUS);
	
	ros::ServiceServer ros_tutorial_service_server = 
						nh.advertiseService("ros_tutorial_srv", calculation);
	
	ROS_INFO("*********ready srv server*************");

	//ros::spin();		// wait for service request
	ros::Rate loop_rate(1);
	while (1)
	{
		nh.getParam("calculation_method", g_operator);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
