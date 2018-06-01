#include "../include/global_planner_qt/movebase.h"
#include "../include/global_planner_qt/moveBaseThread.h"
#include "../include/global_planner_qt/mapthread.h"
#include "../include/global_planner_qt/param.h"
#include "../include/global_planner_qt/main_window.hpp"
#include "../include/global_planner_qt/mapviewer.hpp"
#include <QApplication>
#include <unistd.h>


QMutex mutex;     // global mutex
QWaitCondition condition;
std::vector<std::vector<int> > gridMap;
std::vector<std::pair<int, int> > path;
std::queue<goal_t> goals;
goal_t curr_goal;
int curr_product;
std::vector<std::vector<int> > roboter_local_field;
std::pair<int, int> roboter_pos;
int ROW;
int COL;
double mapResolution;
bool stopSim = true;
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
const int dist_to_resource = 30;
const int dist_to_machine = 30;
std::vector<referee::Machine::Machine_Struct> machines;
std::vector<referee_node::referee_resource> resources;

int main(int argc, char** argv) {
	ROS_INFO("Start ...");

  // Init ros node
  ros::init(argc, argv, "move_base_qt");

  QApplication app(argc, argv);

  /**
   * @brief map_thread to initialize map
   */
  global_planner::MapThread map_thread;
  /**
   * @brief movebase_thread to initialize roboter position
   */
  global_planner::MoveBaseThread movebase_thread;
  std::cout << "debug 1 \n";
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
  global_planner::MainWindow window;
  window.show();
  return app.exec();
}


