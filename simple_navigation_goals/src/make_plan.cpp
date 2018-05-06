#include<ros/ros.h>
#include<nav_msgs/GetPlan.h>
#include<geometry_msgs/PoseStamped.h>

#include<string>
#include<boost/foreach.hpp>
#define forEach BOOST_FOREACH

double g_GoalTolerance = 0.5;
std::string g_WorldFrame = "map";

void fillPathRequest(nav_msgs::GetPlan::Request& req);
void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);

int main(int argc, char** argv) {
	ros::init(argc, argv, "make_pan");

	ros::NodeHandle nh;

	// Init service query for make plan
	std::string service_name = "move_base_node/make_plan";

	// Wait for service to start
	ros::service::waitForService(service_name, ros::Duration(10.0));

	ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
	if (!client) {
		ROS_FATAL("Could not initialize get plan service from %s", client.getService().c_str());
		return -1;
	}

	// Init plan
	nav_msgs::GetPlan srv;
	fillPathRequest(srv.request);

	if (!client) {
		ROS_FATAL("Persistent service connection to %s failed", client.getService().c_str());
		return -1;
	}

	callPlanningService(client, srv);
}

void fillPathRequest(nav_msgs::GetPlan::Request &req) {
	req.start.header.frame_id = g_WorldFrame;
	req.start.pose.position.x = 12.0;
	//req.start.pose.position.y = 5.0;
	req.start.pose.orientation.w = 1.0;

	req.goal.header.frame_id = g_WorldFrame;
	req.goal.pose.position.x = 18.0;
	req.goal.pose.position.y = 30.0;
	req.goal.pose.orientation.w = 1.0;

	req.tolerance = g_GoalTolerance;
}

void callPlanningService(ros::ServiceClient &client, nav_msgs::GetPlan &srv) {
	// Perform the actual path planner call
	if (client.call(srv)) {
		if (!srv.response.plan.poses.empty()) {
			forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) {
				ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
			}
		} else {
			ROS_WARN("Got empty plan");
		}
	} else {
		ROS_ERROR("Fialed to call service %s - is the robot moving?", client.getService().c_str());
	}
}
