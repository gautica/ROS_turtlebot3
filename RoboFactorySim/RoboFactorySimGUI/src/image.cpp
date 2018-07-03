#include "../include/gui/image.h"
#include "../include/gui/param.h"

Image::Image()
{
  sub_image_robot0 = nh.subscribe("/robot_0/robot_camera/image_raw", 1, &Image::processImageDataRobot0, this);
  sub_image_robot1 = nh.subscribe("/robot_1/robot_camera/image_raw", 1, &Image::processImageDataRobot1, this);
  sub_camera_up_robot0 = nh.subscribe("/robot_0/robot_camera_up/image_raw", 1, &Image::processCameraUpRobot0, this);
  sub_camera_up_robot1 = nh.subscribe("/robot_0/robot_camera_up/image_raw", 1, &Image::processCameraUpRobot1, this);
  //sub_image_arena = nh.subscribe("/arena_camera/image_raw", 1, &Image::processImageDataArena, this);
}

void Image::processImageDataRobot0(const sensor_msgs::Image::ConstPtr &image_msg)
{
  if (!init_image_robot0) {
    robot0_buffer = (unsigned char*) malloc(sizeof(unsigned char) * image_msg->data.size());
    init_image_robot0 = true;
    ROS_INFO("Recieved");
  }
  memcpy(robot0_buffer, image_msg->data.data(), image_msg->data.size());

  robot0_width = image_msg->width;
  robot0_height = image_msg->height;
}

void Image::processImageDataRobot1(const sensor_msgs::Image::ConstPtr &image_msg)
{
  if (!init_image_robot1) {
    robot1_buffer = (unsigned char*) malloc(sizeof(unsigned char) * image_msg->data.size());
    init_image_robot1 = true;
    ROS_INFO("Recieved");
  }
  memcpy(robot1_buffer, image_msg->data.data(), image_msg->data.size());
  robot1_width = image_msg->width;
  robot1_height = image_msg->height;
}

void Image::processCameraUpRobot0(const sensor_msgs::Image::ConstPtr &image_msg)
{
  if (!init_camera_up_robot0) {
    robot0_camera_up_buffer = (unsigned char*) malloc(sizeof(unsigned char) * image_msg->data.size());
    init_camera_up_robot0 = true;
    ROS_INFO("Recieved");
  }
  memcpy(robot0_camera_up_buffer, image_msg->data.data(), image_msg->data.size());
  robot0_camera_up_width = image_msg->width;
  robot0_camera_up_height = image_msg->height;
}

void Image::processCameraUpRobot1(const sensor_msgs::Image::ConstPtr &image_msg)
{
  if (!init_camera_up_robot1) {
    robot1_camera_up_buffer = (unsigned char*) malloc(sizeof(unsigned char) * image_msg->data.size());
    init_camera_up_robot1 = true;
    ROS_INFO("Recieved");
  }
  memcpy(robot1_camera_up_buffer, image_msg->data.data(), image_msg->data.size());
  robot1_camera_up_width = image_msg->width;
  robot1_camera_up_height = image_msg->height;
}

/**
void Image::processImageDataArena(const sensor_msgs::Image::ConstPtr &image_msg)
{
  if (!init_image_arena) {
    arena_buffer = (unsigned char*) malloc(sizeof(unsigned char) * image_msg->data.size());
    init_image_arena = true;
    ROS_INFO("Recieved");
  }
  memcpy(arena_buffer, image_msg->data.data(), image_msg->data.size());
  arena_width = image_msg->width;
  arena_height = image_msg->height;
}
*/

