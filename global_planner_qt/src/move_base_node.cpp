#include "movebase.h"
#include "gui_thread.h"
#include <QApplication>
#include "../include/global_planner_qt/main_window.hpp"

int main(int argc, char** argv) {
	ROS_INFO("Start ...");

  // Init ros node
  ros::init(argc, argv, "move_base");

  QApplication app(argc, argv);
  global_planner::MainWindow window;
  global_planner::GUI_Thread gui_thread;
  std::cout << "debug 1 \n";
  gui_thread.start();
  std::cout << "debug 2 \n";
  window.drawMap();
  std::cout << "debug 3 \n";
  window.show();


  return app.exec();
}


