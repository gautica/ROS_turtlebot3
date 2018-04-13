#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
	const double FORWARD_SPEED_MPS = 0.5;

	// Initialize the node
	ros::init(argc, argv, "move_forward");
	ros::NodeHandle nh;

	// A Publisher for the movement data
	ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// Driver forward at a given speed. The robot points up x-axis,
	// The default constructor will set all commands to 0
	geometry_msgs::Twist msg;
	msg.linear.x = FORWARD_SPEED_MPS;

	// Loop a 10Hz, publishing movement commands until we shut down
	ros::Rate rate(10);
	while (ros::ok())
	{
		publisher.publish(msg);
		rate.sleep();
	}
}
