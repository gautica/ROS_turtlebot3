#ifndef MOVEBASE_H
#define MOVEBASE_H
#include"GlobalPlanner.h"
#include"RequestMap.h"
#include<ros/ros.h>
#include<tf/tf.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose2D.h>
#include<nav_msgs/Odometry.h>
#include "../include/global_planner_qt/main_window.hpp"

// coordination <x, y>
typedef std::pair<double, double> CoPair;
#define PI 3.14159265358979323846;
namespace global_planner {
class MoveBase
{
private:
  ros::NodeHandle nh;
  geometry_msgs::Pose2D current_pose;
  ros::Publisher pub_movement;
  ros::Subscriber sub_odom;
  GlobalPlanner planner;
  Pair goal;
public:
  static Pair roboter_pos;
  static std::vector<std::vector<int> > grid;
  static std::vector<std::vector<int> > roboter_local_field;
  static std::vector<Pair> plan;

public:
  MoveBase();
  void excutePlan();
private:
  void init();
  void calc_coordinate_matrix(double x, double y, global_planner::Pair &step);
  void calc_coordiante_map(global_planner::Pair &step, CoPair &coordinate);
  void odom_callback(const nav_msgs::OdometryConstPtr& msg);
  void create_tunnel();
  void find_nearst_pos();
  int calc_distance(Pair point1, Pair point2);
  template<typename T>
  int signur(T num) {
    if (num < 0) return -1;
    else return 1;
  }
};
} // namespace global_planner
#endif // MOVEBASE_H
