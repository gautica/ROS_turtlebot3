#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<turtlesim/Spawn.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_listener");

	ros::NodeHandle nh;

	ros::service::waitForService("spawn");

	ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");

	turtlesim::Spawn srv;
	if (!add_turtle.call(srv)) {
		ROS_ERROR("Failed to call service spawn");
	}

	ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
	tf::TransformListener listener;

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		tf::StampedTransform transform;
		try {
			listener.waitForTransform("/turtle2", "turtle1", ros::Time(0), ros::Duration(10.0));
			listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
		}

		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = 4 * atan2(transform.getOrigin().y(),
									   transform.getOrigin().x());
		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
									  pow(transform.getOrigin().y(), 2));
		turtle_vel.publish(vel_msg);

		loop_rate.sleep();
	}

	return 0;
}
