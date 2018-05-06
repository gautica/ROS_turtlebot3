#include<ros/ros.h>
#include<nav_msgs/GetMap.h>
#include<vector>

// grid map
int rows;
int cols;
double mapResolution;
std::vector<std::vector<int> > grid;

bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printMap();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "load_map");
	ros::NodeHandle nh;

	if (!requestMap(nh)) {
		exit(-1);
	}
	printMap();

	return 0;
}

bool requestMap(ros::NodeHandle &nh)
{
	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;

	while (!ros::service::waitForService("dynamic_map", ros::Duration(3.0)))
	{
		ROS_INFO("Waiting for service to start");
	}

	ROS_INFO("Requesting the map ...");

	ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");

	if (client.call(req, res))
	{
		readMap(res.map);
		return true;
	}

	ROS_ERROR("Failed to call map service");
	return false;
}

void readMap(const nav_msgs::OccupancyGrid& map)
{
	rows = map.info.height;
	cols = map.info.width;
	mapResolution = map.info.resolution;
	ROS_INFO("Received a %d X %d @ %.3f m/p Map\n", rows, cols, mapResolution);

	// Dynamically resize the grid
	grid.resize(rows);
	for (int i = 0; i < rows; i++)
	{
		grid[i].resize(cols);
	}
	int curCell = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			grid[i][j] = map.data[curCell];
			curCell++;
		}
	}
}

void printMap()
{
	std::cout << "Grid map: " << std::endl;
	for (int i = 0; i < rows; i++) {
		printf("Row no. %d\n", i);
		for (int j = 0; j < cols; j++) {
			printf("%d", grid[i][j]);
		}
		printf("\n");
	}
}
