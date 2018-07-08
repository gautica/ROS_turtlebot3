#include "../include/gui/mapthread.h"
#include "../include/gui/param.h"
#include <algorithm>

namespace gui {

MapThread::MapThread()
{
  robot0_map_sub = nh.subscribe("/robot_0/map", 1, &MapThread::updateMap, this);
  robot1_map_sub = nh.subscribe("/robot_1/map", 1, &MapThread::updateMap, this);
  robot0_path_sub = nh.subscribe("/robot_0/robot_path", 1, &MapThread::updateRobot0Path, this);
  robot0_pos_sub = nh.subscribe("/robot_0/odom", 1, &MapThread::updateRobot0Pos, this);
  robot1_map_sub = nh.subscribe("/robot_1/map", 1, &MapThread::updateMap, this);
  robot1_path_sub = nh.subscribe("/robot_1/robot_path", 1, &MapThread::updateRobot1Path, this);
  robot1_pos_sub = nh.subscribe("/robot_1/odom", 1, &MapThread::updateRobot1Pos, this);
}

void MapThread::run()
{
  ros::Rate loop_rate(10);
  while (is_mapViewr_active) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "MapThread stopt ############ \n";
}

void MapThread::updateMap(const nav_msgs::OccupancyGridConstPtr &msg)
{
  ROW = msg->info.height;
  COL = msg->info.width;
  mapResolution = msg->info.resolution;
  if (!is_map_init) {
    gridMap.resize(ROW);
    for (int i = 0; i < ROW; i++) {
      gridMap[i].resize(COL);
      std::fill(gridMap[i].begin(), gridMap[i].end(), -1);
    }
    is_map_init = true;
  }
  int curCell = 0;
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      if (gridMap[i][j] < msg->data[curCell]) {
        gridMap[i][j] = msg->data[curCell];
      }
      curCell++;
    }
  }
}

void MapThread::updateRobot0Pos(const nav_msgs::OdometryConstPtr &msg)
{
  // initialize roboter_pos position as current position
  if (is_map_init) {
    calc_coordinate_matrix(msg -> pose.pose.position.x, msg -> pose.pose.position.y, robot0_pos);
    is_robot0_pos_init = true;
  }

}

void MapThread::updateRobot0Path(const robot_navigation::robot_infoConstPtr &msg)
{
  path_robot0.clear();
  if (msg->path.size() == 0) {
    return;
  }
  for (int i = 0; i < msg->path.size(); i++) {
    path_robot0.push_back(std::make_pair(msg->path[i].y, msg->path[i].x));
  }
  is_robot0_path_init = true;
}

void MapThread::updateRobot1Pos(const nav_msgs::OdometryConstPtr &msg)
{
  if (is_map_init) {
    // initialize roboter_pos position as current position
    calc_coordinate_matrix(msg -> pose.pose.position.x, msg -> pose.pose.position.y, robot1_pos);
    is_robot1_pos_init = true;
  }
}

void MapThread::updateRobot1Path(const robot_navigation::robot_infoConstPtr &msg)
{
  if (msg->path.size() == 0) {
    return;
  }
  path_robot1.clear();
  for (int i = 0; i < msg->path.size(); i++) {
    path_robot1.push_back(std::make_pair(msg->path[i].y, msg->path[i].x));
  }
  is_robot1_path_init = true;
}

void MapThread::calc_coordinate_matrix(double x, double y, std::pair < int, int > & pixel) {
  int origin_x = COL / 2;
  int origin_y = ROW / 2;

  int i = origin_x + (int)(x / mapResolution);
  int j = origin_y + (int)(y / mapResolution);

  pixel.second = i;
  pixel.first = j;
}

}

