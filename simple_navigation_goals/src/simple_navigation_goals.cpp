#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<tf/transform_broadcaster.h>
#include"referee_node/referee_machines.h"

struct referee_info
{
  std::string name;
  float x;
  float y;
};

void referee_callback(referee_node::referee_machinesConstPtr msg);

referee_info ref_info;
int main(int argc, char** argv) {
	ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
	// Create a client for move_base_msgs
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
  // Create subscriber for referee node
  ros::Subscriber sub_referee = nh.subscribe("referee_machines", 10, referee_callback);

  ros::spinOnce();
	// Wait for service to start
	client.waitForServer();

	move_base_msgs::MoveBaseGoal goal;

  // send a goal to robot to move to position 1
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

  // send a goal to robot to move to position 1
  ros::Rate loop_rate(1);
  while (ref_info.x == 0.0) {
    ros::spinOnce();
    ROS_INFO("GOAL.x = %f and GOAL.y = %f", ref_info.x, ref_info.y);
    loop_rate.sleep();
  }
  ROS_INFO("GOAL.x = %f and GOAL.y = %f", ref_info.x, ref_info.y);
  goal.target_pose.pose.position.x = ref_info.x;
  goal.target_pose.pose.position.y = ref_info.y;
	// turn 90 degree
	double theta = 0.0;
	double radians = theta / 180 * M_PI;
	tf::Quaternion quaternion;
	quaternion = tf::createQuaternionFromYaw(radians);
	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);
	goal.target_pose.pose.orientation = qMsg;

	ROS_INFO("Sending goal ...");

	client.sendGoal(goal);

  bool result = client.waitForResult(ros::Duration(30));

	if (result) {
    ROS_INFO("Goal 1 is finished !");
	} else {
		ROS_ERROR("Failed to finish goal");
	}


	return 0;
}

void referee_callback(referee_node::referee_machinesConstPtr msg) {
  ref_info.name = msg->name;
  ref_info.x = msg->x;
  ref_info.y = msg->y;
}
