#ifndef MAPTHREAD_H
#define MAPTHREAD_H

#include <QThread>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose2D.h>
#include<nav_msgs/Odometry.h>
#include "robot_navigation/position.h"
#include "robot_navigation/robot_info.h"

namespace gui {

class MapThread : public QThread
{
public:
  MapThread();

  void run();
private:
void updateMap(const nav_msgs::OccupancyGridConstPtr& msg);
void updateRobot0Path(const robot_navigation::robot_infoConstPtr& msg);
void updateRobot1Path(const robot_navigation::robot_infoConstPtr& msg);
void updateRobot0Pos(const nav_msgs::OdometryConstPtr& msg);
void updateRobot1Pos(const nav_msgs::OdometryConstPtr& msg);
void calc_coordinate_matrix(double x, double y, std::pair <int, int> & pixel);
private:
  ros::NodeHandle nh;
  ros::Subscriber robot0_map_sub;
  ros::Subscriber robot0_path_sub;
  ros::Subscriber robot0_pos_sub;
  ros::Subscriber robot1_map_sub;
  ros::Subscriber robot1_path_sub;
  ros::Subscriber robot1_pos_sub;

};

}


#endif // MAPTHREAD_H
