#ifndef IMAGE_H
#define IMAGE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class Image
{
public:
  Image();

private:
  void processImageDataRobot0(const sensor_msgs::Image::ConstPtr& image_msg);
  void processImageDataRobot1(const sensor_msgs::Image::ConstPtr& image_msg);
  void processCameraUpRobot0(const sensor_msgs::Image::ConstPtr& image_msg);
  void processCameraUpRobot1(const sensor_msgs::Image::ConstPtr& image_msg);
  //void processImageDataArena(const sensor_msgs::Image::ConstPtr& image_msg);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub_image_robot0;
  ros::Subscriber sub_image_robot1;
  ros::Subscriber sub_camera_up_robot0;
  ros::Subscriber sub_camera_up_robot1;
  //ros::Subscriber sub_image_arena;
};



#endif // IMAGE_H
