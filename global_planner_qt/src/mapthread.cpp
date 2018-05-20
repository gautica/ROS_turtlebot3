#include "../include/global_planner_qt/mapthread.h"
#include<geometry_msgs/Twist.h>
#include "../include/global_planner_qt/AStar.h"
#include "../include/global_planner_qt/param.h"
#include <algorithm> // std::fill()

namespace global_planner {

MapThread::MapThread()
{
  this->astar = new AStar;
  map_sub = nh.subscribe("/map", 1, &MapThread::updateMap, this);
  pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  //initMap();
}

MapThread::~MapThread()
{
  delete astar;
}

void MapThread::initMap()
{
  ROS_INFO("Waiting for Initialzation of Map ...");
  while (!is_map_init) {
    ros::spinOnce();
  }
  ROS_INFO("map is initialized ...");
}

void MapThread::run()
{
  initMap();
  while (!is_goal_ready) {
    sleep(1);
  }
  ROS_INFO("Waiting path to initialize ...");
  if (makePlan()) {
    ROS_INFO("Path is initialized");
    is_path_init = true;
  } else {
    is_dest_reachable = false;
    ROS_ERROR("Destination is unreachable!!!");
  }
  updateAll();
}

void MapThread::updateAll()
{
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    calc_costMap();

    if (isPathBlocked() || request_new_plan) {
      if (makePlan()) {
        ROS_INFO("new path is found");
        request_new_plan = false;
      }
    }
    loop_rate.sleep();
  }
}

bool MapThread::makePlan()
{
  mutex.lock();   // lock, because path is also in movebase.cpp used
  std::cout << "in makePlan(), has clock \n";
  pubZeroVel();   // stop roboter while recreate the path
  bool res = this->astar->aStarSearch(gridMap, roboter_pos, curr_goal.goal_pos, path);
  mutex.unlock();
  std::cout << "in makePlan(), release clock \n";
  return res;
}

bool MapThread::isPathBlocked()
{
  //std::cout << "in isPathBlocked() \n";
  for (int i = 0; i < path.size(); i++) {
    if (gridMap[path[i].first][path[i].second] != 0 &&
        gridMap[path[i].first][path[i].second] != -1) {
    //  std::cout << "in isPathBlocked()  return true\n";
      return true;
    }
  }
  //std::cout << "in isPathBlocked() return false\n";
  return false;
}

void MapThread::calc_costMap()
{
  int costMapRange = 15;

  int roboter_row = roboter_pos.first;
  int roboter_col = roboter_pos.second;
  for (int row = roboter_row - 150; row < roboter_row + 150; row++) {
    for (int col = roboter_col - 150; col < roboter_col + 150; col++) {
      if (gridMap[row][col] > 0 && gridMap[row][col] <= 100) {   // Block
        for (int n = -costMapRange; n <= costMapRange; n++) {
          for (int m = -costMapRange; m <= costMapRange; m++) {
            if (gridMap[row+n][col+m] != 100 && gridMap[row+n][col+m] != 110) {
              gridMap[row+n][col+m] = 110;    // 110 indicate costmap
            }
          }
        }
      }
    }
  }
  /**
  for (int i = costMapRange; i < ROW-costMapRange; i++) {
    for (int j = costMapRange; j < COL-costMapRange; j++) {
      if (gridMap[i][j] == 100) {   // Block
        for (int n = -costMapRange; n <= costMapRange; n++) {
          for (int m = -costMapRange; m <= costMapRange; m++) {
            if (gridMap[i+n][j+m] != 100 && gridMap[i+n][j+m] != 110) {
              gridMap[i+n][j+m] = 110;    // 110 indicate costmap
            }
          }
        }
      }
    }
  }
  */
}

void MapThread::updateMap(const nav_msgs::OccupancyGridConstPtr map)
{
  //std::cout << "here in updateMap() \n";
  ROW = map->info.height;
  COL = map->info.width;
  mapResolution = map->info.resolution;
  // Init gridMap, if first get map
  if (!is_map_init) {
    gridMap.resize(ROW);
    for (int i = 0; i < ROW; i++) {
      gridMap[i].resize(COL);
      std::fill(gridMap[i].begin(), gridMap[i].end(), -1);
    }
    is_map_init = true;
  }

  if (is_roboter_pos_init) {
    int roboter_row = roboter_pos.first;
    int roboter_col = roboter_pos.second;
    for (int row = roboter_row - 150; row < roboter_row + 150; row++) {
      for (int col = roboter_col - 150; col < roboter_col + 150; col++) {
        int curCell = COL * row + col;
        gridMap[row][col] = map->data[curCell];
      }
    }
  } else {
    int curCell = 0;
    for (int i = 0; i < ROW; i++) {
      for (int j = 0; j < COL; j++) {
        gridMap[i][j] = map->data[curCell];
        curCell++;
      }
    }
  }
}

void MapThread::pubZeroVel()
{
  geometry_msgs::Twist twist;
  twist.angular.z = 0;
  twist.linear.x = 0;
  pub_vel.publish(twist);
  ros::spinOnce();
}

}   // namespace global_planner

