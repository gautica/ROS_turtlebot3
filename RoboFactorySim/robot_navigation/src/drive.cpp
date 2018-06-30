#include "../include/global_planner_qt/drive.h"
#include <QKeyEvent>
#include <iostream>
#include <geometry_msgs/Twist.h>

namespace global_planner {
Drive::Drive(QKeyEvent* &event) : keyEvent(event)
{
  cmd_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  SPEED_MPS = 0.5;
  Radiant_MPS = 0.5;
}

void Drive::drive()
{
  switch (keyEvent->key()) {
    case Qt::Key_W:
      std::cout << "w is pressed \n";
      move(1);
      break;
    case Qt::Key_S:
      std::cout << "w is pressed \n";
      move(0);
      break;
    case Qt::Key_A:
      turn(1);
      break;
    case Qt::Key_D:
      turn(0);
      break;
    case Qt::Key_Q:
      std::cout << "space\n";
      stop();
      break;
    default:
      break;
  }
}

void Drive::move(bool direction)
{
  geometry_msgs::Twist twist;
  if (direction) {
    twist.linear.x = SPEED_MPS;
  } else {
    twist.linear.x = -SPEED_MPS;
  }
  //ist.angular.z = 0;
  cmd_publisher.publish(twist);
  ros::spinOnce();
  ros::Rate loop_rate(10);
  loop_rate.sleep();
}

void Drive::turn(bool direction)
{
  geometry_msgs::Twist twist;
  if (direction == true) {
    twist.angular.z= Radiant_MPS;
  } else {
    twist.angular.z= -Radiant_MPS;
  }
  twist.linear.x = 0;
  cmd_publisher.publish(twist);
  ros::spinOnce();
  ros::Rate loop_rate(10);
  loop_rate.sleep();
}

void Drive::stop()
{
  geometry_msgs::Twist twist;
  twist.linear.x = 0;
  twist.angular.z = 0;
  cmd_publisher.publish(twist);
  ros::spinOnce();
  ros::Rate loop_rate(10);
  loop_rate.sleep();
}

}

