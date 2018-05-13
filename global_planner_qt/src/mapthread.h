#ifndef MAPTHREAD_H
#define MAPTHREAD_H

#include <QThread>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace global_planner {

class AStar;
class MoveBase;
class MapThread : public QThread
{
private:
  AStar* astar;
public:
  const static double MIN_SCAN_ANGLE_RAD = 30.0 / 180.0 * M_PI;
  const static float MIN_PROXIMITY_RANGE_M = 0.3;
  static std::vector<std::vector<int> > gridMap;
  static std::vector<std::pair<int, int> > path;
  static int ROW;
  static int COL;
  static double mapResolution;
  bool isMapInit;
  bool isPathInit;

private:
  ros::NodeHandle nh;
  ros::Subscriber map_sub;
  ros::Publisher pub_vel;
public:
  MapThread();
  ~MapThread();
  void run();
  void makePlan();
private:
  void updateMap(const nav_msgs::OccupancyGridConstPtr map);
  void calc_costMap();
  bool isPathBlocked();
  void updateAll();
  void pubZeroVel();
};
}


#endif // MAPTHREAD_H
