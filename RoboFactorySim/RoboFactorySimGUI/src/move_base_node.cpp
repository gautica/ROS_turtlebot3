#include "../include/gui/param.h"
#include "../include/gui/mainwindow.hpp"
#include "../include/gui/image.h"
#include "RoboFactorySimGUI/Product.h"
#include <QApplication>
#include <pthread.h>
#include <ros/ros.h>

int curr_product = NO_PRODUCT;
std::vector<int> products;
std::vector<int> products_copy;
//bool stopSim = false;
uchar* robot0_buffer;
unsigned int robot0_width = 0;
unsigned int robot0_height = 0;
uchar* robot1_buffer;
unsigned int robot1_width = 0;
unsigned int robot1_height = 0;
uchar* arena_buffer;
unsigned int arena_width = 0;
unsigned int arena_height = 0;
bool init_image_robot1 = false;
bool init_image_robot0 = false;
bool init_image_arena = false;
int curr_gamemode = KI_VS_KI;
/**
 * @brief assign_product
 * @param request, contains robot id
 * @param response
 * @return
 */
bool assign_product(RoboFactorySimGUI::Product::Request &request,
                    RoboFactorySimGUI::Product::Response &response)
{
  switch (curr_gamemode) {
  case KI_VS_KI:
    if (request.request == 0) {   // robot0
      if (!products.empty()) {
        response.result = products[products.size() - 1];
        products.pop_back();
      } else {
        response.result = NO_PRODUCT;
      }
    } else {    // robot1
      if (!products_copy.empty()) {
        response.result = products_copy[products_copy.size() - 1];
        products_copy.pop_back();
      } else {
        response.result = NO_PRODUCT;
      }
    }
    break;
  case PLAYER_VS_PLAYER:

    break;
  case AS_TEAM:
    if (!products.empty()) {
      response.result = products[products.size() - 1];
      products.pop_back();
    } else {
      response.result = NO_PRODUCT;
    }
    break;
  default:
    break;
  }


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


