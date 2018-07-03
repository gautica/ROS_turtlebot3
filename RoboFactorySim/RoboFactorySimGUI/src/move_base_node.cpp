#include "../include/gui/param.h"
#include "../include/gui/mainwindow.hpp"
#include "../include/gui/gamewindow.hpp"
#include "../include/gui/image.h"
#include "RoboFactorySimGUI/Product.h"
#include <QApplication>
#include <pthread.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>

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

bool update_status = false;

// read products.txt and gamemode.txt
bool read_config()
{
  std::string gamemodefile = "/home/sep_2018/yan/Qt_Projects/build-RoboFactorySim-Desktop_Qt_5_11_0_GCC_64bit-Debug/gamemode.txt";
  std::ifstream file1(gamemodefile.c_str());
  std::stringstream buffer1;
  buffer1 << file1.rdbuf();
  std::string str = buffer1.str();
  curr_gamemode = std::stoi(str);
  std::cout << "game mode: " << curr_gamemode << "\n";

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
      }
    } else {    // robot1
      if (!products_copy.empty()) {
        response.result = products_copy[products_copy.size() - 1];
        curr_product_robo1 = products_copy[products.size() - 1];
        products_copy.pop_back();
      } else {
        response.result = NO_PRODUCT;
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

int main(int argc, char** argv) {
	ROS_INFO("Start ...");

  read_config();

  // Init ros node
  ros::init(argc, argv, "RoboFactorySimGUI_node");
  QApplication app(argc, argv);
  pthread_t service_thread;
  int status = pthread_create(&service_thread, NULL, &create_service, NULL);
  if (status) {
    std::cout << "Error by creating new thread \n";
    exit(-1);
  } else {
    std::cout << "Successfully created new thread\n";
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
  gui::MainWindow window;
  window.show();

  return app.exec();
}


