/*
 * RequestMap.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: yashuai
 */

#include "RequestMap.h"

namespace global_planner {


int RequestMap::ROW = 0;
int RequestMap::COL = 0;
double RequestMap::mapResolution = 0.0;

RequestMap::RequestMap() {
	// TODO Auto-generated constructor stub

}

RequestMap::~RequestMap() {
	// TODO Auto-generated destructor stub
}

bool RequestMap::requestMap(std::vector<std::vector<int> > &grid)
{
	ros::NodeHandle nh;
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
		readMap(res.map, grid);
		return true;
	}

	ROS_ERROR("Failed to call map service");
	return false;
}

void RequestMap::readMap(const nav_msgs::OccupancyGrid& map, std::vector<std::vector<int> > &grid)
{
	ROW = map.info.height;
	COL = map.info.width;
	mapResolution = map.info.resolution;
	ROS_INFO("Received a %d X %d @ %.3f m/p Map\n", ROW, COL, mapResolution);

	// Dynamically resize the grid
	grid.resize(ROW);
	for (int i = 0; i < ROW; i++)
	{
		grid[i].resize(COL);
	}
	int curCell = 0;
	for (int i = 0; i < ROW; i++) {
		for (int j = 0; j < COL; j++) {
			grid[i][j] = map.data[curCell];
			curCell++;
		}
	}
}
} /* namespace global_planner */

