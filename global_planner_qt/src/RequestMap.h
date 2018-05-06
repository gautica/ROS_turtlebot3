/*
 * RequestMap.h
 *
 *  Created on: Apr 25, 2018
 *      Author: yashuai
 */
#ifndef REQUESTMAP_H_
#define REQUESTMAP_H_

#include<ros/ros.h>
#include<nav_msgs/GetMap.h>

namespace global_planner {

class RequestMap {
public:
	static int ROW;
	static int COL;
	static double mapResolution;
public:
	RequestMap();
	virtual ~RequestMap();
  static bool requestMap(std::vector<std::vector<int> > &grid);
  static void readMap(const nav_msgs::OccupancyGrid& msg, std::vector<std::vector<int> > &grid);
};

} /* namespace global_planner */

#endif /* REQUESTMAP_H_ */
