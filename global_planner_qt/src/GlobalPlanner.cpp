/*
 * GlobalPlanner.cpp
 *
 *  Created on: Apr 22, 2018
 *      Author: yashuai
 */

#include "GlobalPlanner.h"
#include "RequestMap.h"
namespace global_planner {

GlobalPlanner::GlobalPlanner() {
	// TODO Auto-generated constructor stub

}

GlobalPlanner::~GlobalPlanner() {
	// TODO Auto-generated destructor stub
}

bool GlobalPlanner::makePlan(Pair& start,
							 const Pair& goal,
               std::vector<std::vector<int> > &grid,
							 std::vector<Pair>& plan) {

	// Firstly request dynamic map
  if (!RequestMap::requestMap(grid)) {
		ROS_ERROR("Failed to request map !!!");
		exit(1);
	}

	this->astar.aStarSearch(grid, start, goal, plan);

	return true;
}

} /* namespace global_planner */
