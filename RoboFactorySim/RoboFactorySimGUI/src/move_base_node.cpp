#include "../include/gui/param.h"
#include "../include/gui/mainwindow.hpp"
#include "../include/gui/gamewindow.hpp"
#include "../include/gui/image.h"
#include "RoboFactorySimGUI/Product.h"
#include "roboter_controller/Status.h"
#include <QApplication>
#include <pthread.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>

pthread_t player_thread;
pthread_t service_thread;

std::vector<std::vector<int> > gridMap;
std::vector<std::pair<int, int> > path_robot0;
std::vector<std::pair<int, int> > path_robot1;
std::vector<std::vector<int> > roboter_local_field;
std::pair<int, int> robot0_pos;
std::pair<int, int> robot1_pos;

int curr_product_robo0 = NO_PRODUCT;
int curr_product_robo1 = NO_PRODUCT;
std::vector<int> products;
std::vector<int> products_copy;
//bool stopSim = false;
uchar* robot0_buffer;
unsigned int robot0_width = 0;
unsigned int robot0_height = 0;

uchar* robot0_camera_up_buffer;
unsigned int robot0_camera_up_width = 0;
unsigned int robot0_camera_up_height = 0;

uchar* robot1_buffer;
unsigned int robot1_width = 0;
unsigned int robot1_height = 0;

uchar* robot1_camera_up_buffer;
unsigned int robot1_camera_up_width = 0;
unsigned int robot1_camera_up_height = 0;

bool init_image_robot1 = false;
bool init_image_robot0 = false;
bool init_camera_up_robot0 = false;
bool init_camera_up_robot1 = false;
//bool init_image_arena = false;
int curr_gamemode = KI_VS_KI;
int ROW = 0;
int COL = 0;
double mapResolution = 0;
bool update_status = false;
bool is_map_init = false;
bool is_robot0_pos_init = false;
bool is_robot1_pos_init = false;
bool is_robot0_path_init = false;
bool is_robot1_path_init = false;
bool is_mapViewr_active = false;

bool go = false;
bool player1_topview = false;
bool player2_topview = false;

bool init_products = false;
bool is_finished_robot0 = false;
bool is_finished_robot1 = false;
int who_wins = -1;
// read products.txt and gamemode.txt
void read_config()
{
  std::string gamemodefile = "/home/sep_2018/yan/Qt_Projects/build-RoboFactorySim-Desktop_Qt_5_11_0_GCC_64bit-Debug/gamemode.txt";
  std::ifstream file1(gamemodefile.c_str());
  std::stringstream buffer1;
  buffer1 << file1.rdbuf();
  std::string str = buffer1.str();
  curr_gamemode = std::stoi(str);
  std::cout << "game mode: " << curr_gamemode << "\n";

  if (curr_gamemode == PLAYER_VS_PLAYER) {
    return;
  }
  std::string productsfile = "/home/sep_2018/yan/Qt_Projects/build-RoboFactorySim-Desktop_Qt_5_11_0_GCC_64bit-Debug/products.txt";
  std::ifstream file(productsfile.c_str());
  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();
  str = buffer.str();
  std::cout << "read products.txt: " << str;

  std::istringstream iss(str);
  std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                   std::istream_iterator<std::string>());
  for (int i = 0; i < results.size(); i++) {
    products.push_back(std::stoi(results[i]));
  }
  for (int i = 0; i < products.size(); i++) {
    std::cout << "products[" << i << "]: " << products[i] << "\n";
  }
  if (curr_gamemode == KI_VS_KI) {
    products_copy = products;
  }

}

bool assign_product(RoboFactorySimGUI::Product::Request &request,
                    RoboFactorySimGUI::Product::Response &response)
{
  switch (curr_gamemode) {
  case KI_VS_KI:
    if (request.request == 0) {   // robot0
      if (!products.empty()) {
        response.result = products[products.size() - 1];
        curr_product_robo0 = products[products.size() - 1];
        products.pop_back();
      } else {
        response.result = NO_PRODUCT;
        is_finished_robot0 = true;
        who_wins = ROBOT_0;
      }
    } else {    // robot1
      if (!products_copy.empty()) {
        response.result = products_copy[products_copy.size() - 1];
        curr_product_robo1 = products_copy[products_copy.size() - 1];
        products_copy.pop_back();
      } else {
        response.result = NO_PRODUCT;
        is_finished_robot1 = true;
        who_wins = ROBOT_1;
      }
    }
    break;
  case AS_TEAM:
    if (!products.empty()) {
      if (request.request == 0) {
        curr_product_robo0 = products[products.size() - 1];
      } else {
        curr_product_robo1 = products[products.size() - 1];
      }
      response.result = products[products.size() - 1];
      products.pop_back();
    } else {
      response.result = NO_PRODUCT;
      if (request.request == 0) {
        is_finished_robot0 = true;
      } else {
        is_finished_robot1 = true;
      }

    }
    break;
  default:
    break;
  }

  update_status = true;
  return true;
}

void* create_service(void*)
{
  ros::NodeHandle nh;
  ros::ServiceServer product_server = nh.advertiseService("product_assignment", &assign_product);

  ROS_INFO ("************** ready assign product service *******************");
  ros::spin();
}

void gamepad_status_callback(const roboter_controller::StatusConstPtr& msg)
{
  go = msg->go;
  player1_topview = msg->player1_topview;
  player2_topview = msg->player2_topview;
  if (!init_products) {   // only one time
    for (int i = 0; i < msg->tasks.size(); i++) {
      products.push_back(msg->tasks[i]);
    }
    init_products = true;
  }
  products_copy = products;
  update_status = true;
  for (int i = 0; i < products.size(); i++) {
    std::cout << "products[" << i << "]: " << products[i] << "\n";
  }

}

void* gamepad_status(void*)
{
  ros::NodeHandle nh;
  ros::Subscriber status_sub = nh.subscribe("/status", 1, gamepad_status_callback);
  ROS_INFO ("************** ready receive status from gamepad *******************");
  //ros::spin();
  //int res;
  pthread_join(service_thread, NULL);
}

int main(int argc, char** argv) {
	ROS_INFO("Start ...");

  read_config();

  // Init ros node
  ros::init(argc, argv, "RoboFactorySimGUI_node");
  QApplication app(argc, argv);

  int status = pthread_create(&service_thread, NULL, &create_service, NULL);
  if (status) {
    ROS_ERROR("Error by creating new thread");
    exit(-1);
  } else {
    ROS_INFO("Successfully created new thread");
  }

  if (curr_gamemode == PLAYER_VS_PLAYER) {

    if (pthread_create(&player_thread, NULL, &gamepad_status, NULL)) {
      ROS_ERROR("Error by creating new thread");
      exit(-1);
    } else {
      ROS_INFO("Successfully created new thread");
    }
  }
  Image image;


  /**
  while (!init_image_robot0 || !init_image_robot1 || !init_image_arena) {
    sleep(1);
  }
  */
  /**
   * @brief window to show dynamical map
   */
  gui::GameWindow window;
  window.show();

  return app.exec();
}


