/*
 * GlobalPlanner.h
 *
 *  Created on: Apr 22, 2018
 *      Author: yashuai
 */
#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_


#include"AStar.h"

using std::string;

namespace global_planner {

class GlobalPlanner {
private:
  AStar astar;
public:
	GlobalPlanner();
	virtual ~GlobalPlanner();

  bool makePlan(Pair& start,
				  const Pair& goal,
          std::vector<std::vector<int> > &grid,
				  std::vector<Pair> &plan);
};

} /* namespace global_planner */

#endif /* GLOBALPLANNER_H_ */
