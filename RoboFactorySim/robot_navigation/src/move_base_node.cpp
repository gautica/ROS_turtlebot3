#include "../include/global_planner_qt/movebase.h"
#include "../include/global_planner_qt/moveBaseThread.h"
#include "../include/global_planner_qt/mapthread.h"
#include "../include/global_planner_qt/param.h"
#include "../include/global_planner_qt/mapviewer.hpp"
#include "../include/global_planner_qt/routing.h"
#include "robot_navigation/Product.h"
#include "robot_navigation/robot_info.h"
#include "robot_navigation/position.h"
#include <QApplication>
#include <unistd.h>


QMutex mutex;     // global mutex
QWaitCondition condition;
std::vector<std::vector<int> > gridMap;
std::vector<std::pair<int, int> > path;
std::queue<goal_t> goals;
goal_t curr_goal;
int curr_product = NO_PRODUCT;
int curr_location = WAREHOUSE;
std::vector<std::vector<int> > roboter_local_field;
std::pair<int, int> roboter_pos;
int ROW;
int COL;
double mapResolution;
bool stopSim = false;
bool is_dest_reachable = true;
bool is_roboter_pos_init = false;
bool is_job_finished = false;
bool request_new_plan = false;
bool is_map_init = false;
bool is_path_init = false;
bool grabbed = false;
bool is_reach_goal = false;
bool is_goal_ready = false;

const int costMap_area = 300;
const int dist_to_resource = 28;
const int dist_to_machine = 30; //30
std::vector<global_planner::Referee::Machine_Struct> machines;
std::vector<referee_node::referee_resource> resources;
std::vector<int> products;
std::string will_grab;
std::string roboter_name;
bool is_navigate = false;
int machine_id = -1;

int main(int argc, char** argv) {
	ROS_INFO("Start ...");

  roboter_name = argv[1];
  std::cout << "roboter_name: " << roboter_name << "\n";

  for (int i = 0; i < argc; i++) {
    std::cout << "argv: " << argv[i] <<"\n";
  }
  // Init ros node
  ros::init(argc, argv, "roboter_name");
  ros::NodeHandle nh;

  /**
   * @brief map_thread to initialize map
   */
  global_planner::MapThread map_thread;
  /**
   * @brief movebase_thread to initialize roboter position
   */
  global_planner::MoveBaseThread movebase_thread;

  map_thread.start();
/**
  while (!is_path_init) {
    sleep(1);
  }
  */
  movebase_thread.start();
    /**
   * @brief window to show dynamical map
   */
  while (!is_map_init) {
    sleep(1);
  }

  ros::ServiceClient client = nh.serviceClient<robot_navigation::Product>("product_assignment");
  robot_navigation::Product service;
  if (roboter_name == "robot_0") {
    service.request.request = 0;
  } else if (roboter_name == "robot_1") {
    service.request.request = 1;
  } else {
    ROS_ERROR("Unvailid robot name, should be robot_0 or robot_1");
    exit(-1);
  }
  while (curr_product == -1) {
    if (client.call(service)) {
      ROS_INFO("Request service sucessfully, receieve product number %ld", service.response.result);
      curr_product = service.response.result;
    }
    sleep(1);
  }


  global_planner::Routing routing;
  routing.make_plan();

  //global_planner::MapViewer window;
  //window.show();
  //return app.exec();
  ros::Publisher pub_path = nh.advertise<robot_navigation::robot_info>(roboter_name + "/robot_path", 1);
  robot_navigation::robot_info path_info;
  robot_navigation::position pos;
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    if (path.size() == 0) {
      sleep(1);
    }
    path_info.path.clear();
    for (int i = 0; i < path.size(); i++) {
      pos.x = path[i].second;
      pos.y = path[i].first;
      path_info.path.push_back(pos);
    }
    pub_path.publish(path_info);
    loop_rate.sleep();
  }

  movebase_thread.wait();
  map_thread.wait();
  return 0;
}


