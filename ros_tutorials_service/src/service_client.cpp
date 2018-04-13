#include<ros/ros.h>
#include"ros_tutorials_service/SrvTutorial.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "service_client");
	
	if (argc != 3)
 	{
		ROS_INFO("cmd: rosrun ros_tutorials_service service_client arg0, arg1");
		ROS_INFO("arg0 and arg1: double number");
		return 1;
	}

	ros::NodeHandle nh;

	ros::ServiceClient ros_tutorial_service_client =
			nh.serviceClient<ros_tutorials_service::SrvTutorial>("ros_tutorial_srv");

	
	ros_tutorials_service::SrvTutorial service;
	
	service.request.req1 = atoll(argv[1]);
	service.request.req2 = atoll(argv[2]);

	// Request the service, if the request ist accepted, display the response value
	if (ros_tutorial_service_client.call(service))
	{
		ROS_INFO("send service, service.request.req1 and req2: %ld, %ld", 
				(long int)service.request.req1, (long int)service.request.req2);
		ROS_INFO("receive service, service.response.result: %ld", 
				(long int)service.response.result);
	}
	else
	{
		ROS_INFO("Failed to call service ros_tutorial_srv");
		return 1;
	}
	
	return 0;
}
