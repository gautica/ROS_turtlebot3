#ifndef DRIVE_H
#define DRIVE_H

#include <ros/ros.h>

class QKeyEvent;
namespace global_planner {
class Drive
{
private:
  QKeyEvent* keyEvent;
  ros::NodeHandle nh;
  ros::Publisher cmd_publisher;
  float SPEED_MPS;
  float Radiant_MPS;

public:
  Drive(QKeyEvent* &event);
  void drive();

private:
  void move(bool direction);
  void turn(bool  direction);
  void stop();
};

} // namespace global_planner


#endif // DRIVE_H
