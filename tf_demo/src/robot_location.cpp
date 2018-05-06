#include<ros/ros.h>
#include<tf/transform_listener.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_location");
	ros::NodeHandle nh;

	tf::TransformListener listener;
	ros::Rate loop_rate(2.0);
	listener.waitForTransform("/base_footprint", "/map", ros::Time(0), ros::Duration(10.0));

	while (ros::ok()) {
		tf::StampedTransform transform;
		try {
			listener.lookupTransform("/base_footprint", "/map", ros::Time(0), transform);
			double x = transform.getOrigin().x();
			double y = transform.getOrigin().y();
			std::cout << "Current position: (" << x << ", " << y << ")" << std::endl;
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
		}
		loop_rate.sleep();
	}
}
