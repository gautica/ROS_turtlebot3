#include"Stopper.h"

int main(int argc, char** argv)
{
	// Initialize new ros node named "stopper"
	ros::init(argc, argv, "stopper");

	// Create new stopper object
	Stopper stopper;
	stopper.startMoving();

	return 0;
}
