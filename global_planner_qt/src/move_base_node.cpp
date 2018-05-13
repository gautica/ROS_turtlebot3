#include "movebase.h"
#include "gui_thread.h"
#include "mapthread.h"
#include <QApplication>
#include "../include/global_planner_qt/main_window.hpp"
#include <unistd.h>
#include "mutex.h"

QMutex mutex;     // global mutex
QWaitCondition condition;
bool quitSim = true;

int main(int argc, char** argv) {
	ROS_INFO("Start ...");

  // Init ros node
  ros::init(argc, argv, "move_base");

  QApplication app(argc, argv);

  global_planner::MapThread map_thread;
  global_planner::GUI_Thread gui_thread;
  map_thread.start();
  std::cout << "debug 1 \n";
  while (!map_thread.isPathInit) {
    sleep(1);
  }
  gui_thread.start();
  std::cout << "debug 2 \n";
  //window.drawMap();
  //std::cout << "debug 3 \n";

  global_planner::MainWindow window;
  window.show();


  return app.exec();
}


