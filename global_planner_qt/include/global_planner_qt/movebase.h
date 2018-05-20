#ifndef MOVEBASE_H
#define MOVEBASE_H
#include<ros/ros.h>
#include<tf/tf.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose2D.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/LaserScan.h>
/**
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "global_planner_qt/got_resource.h"

//#include <string>
#include <stdlib.h>
#include <iostream>
*/
// coordination <x, y>
typedef std::pair<double, double> CoPair;
namespace global_planner {
class MoveBase
{
  /**
  struct Resource
{
    std::string name;
    float x;
    float y;
    float distance;
};
*/
  enum MoveDirection {
    BACKWARD = -1,
    FORWARD = 1
  };
  enum RotateDirection {
    CLOCKWISE = -1,
    ANTICLOCKWISE = 1
  };
  enum SPEED_MPS {
    SLOW = 3,
    FAST = 5
  };
  enum STEPS {
    SHORT = 5,
    LONG = 10
  };
private:
  ros::NodeHandle nh;
  geometry_msgs::Pose2D current_pose;
  ros::Publisher pub_movement;
  ros::Subscriber sub_odom;
  ros::Subscriber sub_laser;
  /**
  ros::Publisher publisher;
  ros::Subscriber sub;
  ros::Subscriber sub_resource;
  ros::Subscriber sub_resource_position;
  */
  MoveDirection moveDirection;
  RotateDirection rotateDirection;
  SPEED_MPS speed;
  STEPS steps;
  /**
  int sub_counter;
  Resource resources[3];
  std::string current_resource;
  int current_publish_id;
  bool found_ressources;
  int resources_in_simulation;
 */
public:
  const static double MIN_SCAN_ANGLE_RAD = 30.0 / 180.0 * M_PI;
  const static float MIN_PROXIMITY_RANGE_M = 0.5;

public:
  MoveBase();
  void start();

private:
  void init();
  void excutePlan();
  void calc_coordinate_matrix(double x, double y, std::pair<int, int> &step);
  void calc_coordinate_map(std::pair<int, int> &step, CoPair &coordinate);
  void odom_callback(const nav_msgs::OdometryConstPtr& msg);
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void find_nearst_pos();
  int calc_distance(std::pair<int, int> point1, std::pair<int, int> point2);
  float calc_movement();
  bool reach_goal(geometry_msgs::Twist &twist);
  void rotate(geometry_msgs::Twist &twist);
  void move(geometry_msgs::Twist &twist);
  void rotate_to_goal(geometry_msgs::Twist &twist);
  void pubZeroVel(geometry_msgs::Twist &twist);
  //void grab_resource();
/**
  void pose_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg);
  void resource_callback(const global_planner_qt::got_resource::ConstPtr& msg);
  void resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
*/
  template<typename T>
  int signur(T num) {
    if (num < 0) return -1;
    else return 1;
  }

  template<typename T>
  T abs(T x) {
    return signur(x) * x;
  }
};
} // namespace global_planner
#endif // MOVEBASE_H
